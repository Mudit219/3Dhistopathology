
#include "utils/structs.glsl"
#include "utils/sampler2d.glsl"
#include "utils/sampler3d.glsl"

#include "utils/classification.glsl"
#include "utils/compositing.glsl"
#include "utils/depth.glsl"
#include "utils/gradients.glsl"
#include "utils/shading.glsl"
#include "utils/raycastgeometry.glsl"

// #include "utils/isosurface.glsl"

uniform VolumeParameters volumeParameters;
uniform sampler3D volume;

// uniform sampler2D transferFunction;

uniform ImageParameters entryParameters;
uniform sampler2D entryColor;
uniform sampler2D entryDepth;
uniform sampler2D entryPicking;
uniform sampler2D entryNormal;

uniform ImageParameters exitParameters;
uniform sampler2D exitColor;
uniform sampler2D exitDepth;

uniform ImageParameters bgParameters;
uniform sampler2D bgColor;
uniform sampler2D bgPicking;
uniform sampler2D bgDepth;

uniform ImageParameters outportParameters;

uniform LightParameters lighting;
uniform CameraParameters camera;
uniform VolumeIndicatorParameters positionindicator;
uniform RaycastingParameters raycaster;
// uniform IsovalueParameters isovalues;

uniform bool useNormals = false;

uniform int channel;

uniform vec4 pointValue;


#define ERT_THRESHOLD 0.99  // threshold for early ray termination

#if (!defined(INCLUDE_DVR) && !defined(INCLUDE_ISOSURFACES))
#  define INCLUDE_DVR
#endif

vec4 getColorVal(vec4 colorPoint, vec4 voxel)
{
    if(voxel.r > colorPoint.r - 0.1 && voxel.r < colorPoint.r + 0.1)
    {
        if(voxel.g > colorPoint.g - 0.1 && voxel.g < colorPoint.g + 0.1)
        {
            if(voxel.b > colorPoint.b - 0.1 && voxel.b < colorPoint.b + 0.1)
            {
                return voxel;
            }
        }
    }
    return vec4(0.0,0.0,0.0,0.0);
}

vec4 getColorVal(vec4 colorPoint, vec4 voxel, int channel)
{
    if(voxel[channel] > colorPoint[channel] - 0.1 && voxel[channel] < colorPoint[channel] + 0.1)
    {
        switch(channel){
            case 0: return vec4(voxel[0],0.0,0.0,0.0);
            case 1: return vec4(0.0,voxel[1],0.0,0.0);
            case 2: return vec4(0.0,0.0,voxel[2],0.0);
        }
    }
    return vec4(0.0,0.0,0.0,0.0);
}

vec4 rayTraversal(vec3 entryPoint, vec3 exitPoint, vec2 texCoords, float backgroundDepth, vec3 entryNormal) {
    vec4 result = vec4(0.0);
    vec3 rayDirection = exitPoint - entryPoint;
    float tEnd = length(rayDirection);
    float tIncr = min(
        tEnd, tEnd / (raycaster.samplingRate * length(rayDirection * volumeParameters.dimensions)));
    float samples = ceil(tEnd / tIncr);
    tIncr = tEnd / samples;
    float t = 0.5f * tIncr;
    rayDirection = normalize(rayDirection);
    float tDepth = -1.0;
    vec4 color;
    vec4 voxel;
    vec3 samplePos;
    vec3 toCameraDir = normalize((volumeParameters.textureToWorld * vec4(entryPoint, 1.0) -
                                  volumeParameters.textureToWorld * vec4(exitPoint, 1.0))
                                     .xyz);

    vec4 backgroundColor = vec4(0);
    float bgTDepth = -1;
    #ifdef BACKGROUND_AVAILABLE
        backgroundColor = texture(bgColor, texCoords);
        // convert to raycasting depth
        bgTDepth = tEnd * calculateTValueFromDepthValue(
            camera, backgroundDepth, texture(entryDepth, texCoords).x, texture(exitDepth, texCoords).x);        

        if (bgTDepth < 0) {
            result = backgroundColor;
        }
    #endif // BACKGROUND_AVAILABLE

    // used for isosurface computation
    voxel = getNormalizedVoxel(volume, volumeParameters, entryPoint + t * rayDirection);


    bool first = true;
    while (t < tEnd) {
        samplePos = entryPoint + t * rayDirection;
        vec4 previousVoxel = voxel;
        voxel = getNormalizedVoxel(volume, volumeParameters, samplePos);

    #if defined(BACKGROUND_AVAILABLE)
            result = DRAW_BACKGROUND(result, t, tIncr, backgroundColor, bgTDepth, tDepth);
    #endif // BACKGROUND_AVAILABLE

    #if defined(PLANES_ENABLED)
            result = DRAW_PLANES(result, samplePos, rayDirection, tIncr, positionindicator, t, tDepth);
    #endif // #if defined(PLANES_ENABLED)

    #if defined(INCLUDE_DVR)
            // color = APPLY_CHANNEL_CLASSIFICATION(transferFunction, voxel, channel);
            color = APPLY_CLASSIFICATION(pointValue, voxel);
            if (color.a > 0) {
                
                vec3 gradient;
                if (first && useNormals) {
                    gradient = -entryNormal;
                } else {
                    gradient = COMPUTE_GRADIENT_FOR_CHANNEL(voxel, volume, volumeParameters, samplePos, channel);
                    gradient = normalize(gradient);
                }
                // World space position
                vec3 worldSpacePosition = (volumeParameters.textureToWorld * vec4(samplePos, 1.0)).xyz;
                // Note that the gradient is reversed since we define the normal of a surface as
                // the direction towards a lower intensity medium (gradient points in the increasing
                // direction)
                color.rgb = APPLY_LIGHTING(lighting, color.rgb, color.rgb, vec3(1.0),
                                        worldSpacePosition, -gradient, toCameraDir);
                

                result = APPLY_COMPOSITING(result, color, samplePos, voxel, gradient, camera,
                                        raycaster.isoValue, t, tDepth, tIncr);
            }
    #endif // INCLUDE_DVR

        // early ray termination
        if (result.a > ERT_THRESHOLD) {
            t = tEnd;
        } else {
            t += tIncr;
        }
        first = false;
    }

    // composite background if lying beyond the last volume sample, which is located at tEnd - tIncr*0.5
    if (bgTDepth > tEnd - tIncr * 0.5) {
        result =
            DRAW_BACKGROUND(result, bgTDepth, tIncr * 0.5, backgroundColor, bgTDepth, tDepth);
    }

    if (tDepth != -1.0) {
        tDepth = calculateDepthValue(camera, tDepth / tEnd, texture(entryDepth, texCoords).x,
                                     texture(exitDepth, texCoords).x);

    } else {
        tDepth = 1.0;
    }

    gl_FragDepth = min(backgroundDepth, tDepth);

    return result;
}

void main() {
    vec2 texCoords = gl_FragCoord.xy * outportParameters.reciprocalDimensions;
    vec3 entryPoint = texture(entryColor, texCoords).rgb;
    vec3 exitPoint = texture(exitColor, texCoords).rgb;

    vec4 color = vec4(0);

    float backgroundDepth = 1;
#ifdef BACKGROUND_AVAILABLE
    color = texture(bgColor, texCoords);
    gl_FragDepth = backgroundDepth = texture(bgDepth, texCoords).x;
    PickingData = texture(bgPicking, texCoords);
#else // BACKGROUND_AVAILABLE
    PickingData = vec4(0);
    if (entryPoint == exitPoint) {
        discard;
    }
#endif // BACKGROUND_AVAILABLE

    vec3 normal = useNormals ? texture(entryNormal, texCoords).xyz : vec3(0,0,0);
    if (entryPoint != exitPoint) {
        color = rayTraversal(entryPoint, exitPoint, texCoords, backgroundDepth, normal);
    }
    FragData0 = color;
}