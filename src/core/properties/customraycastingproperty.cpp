
#include <inviwo/core/properties/customraycastingproperty.h>

namespace inviwo {

const std::string CustomRaycastingProperty::classIdentifier = "org.inviwo.CustomRaycastingProperty";
std::string CustomRaycastingProperty::getClassIdentifier() const { return classIdentifier; }

CustomRaycastingProperty::CustomRaycastingProperty(std::string identifier, std::string displayName,
                                       InvalidationLevel invalidationLevel,
                                       PropertySemantics semantics)
    : CompositeProperty(identifier, displayName, invalidationLevel, semantics)
    , renderingType_("renderingType", "Rendering",
                     {{"dvr", "Direct Volume Rendering", RenderingType::Dvr}},
                     0, InvalidationLevel::InvalidResources)
    , classification_("classificationMode", "Classification",
                      {{"none", "None", Classification::None},
                       {"transfer-function", "Transfer Function", Classification::TF},
                       {"voxel-value", "Voxel Value", Classification::Voxel}},
                      1, InvalidationLevel::InvalidResources)
    , compositing_(
          "compositingMode", "Compositing",
          {{"dvr", "Direct Volume Rendering", CompositingType::Dvr},
           {"mip", "Maximum Intensity Projection (MIP)", CompositingType::MaximumIntensity},
           {"fhp", "First Hit (Points)", CompositingType::FirstHitPoints},
           {"fhn", "First Hit (Normals)", CompositingType::FirstHitNormals},
           {"fhnvs", "First Hit (View Space Normals)", CompositingType::FirstHistNormalsView},
           {"fhd", "First Hit (Depth)", CompositingType::FirstHitDepth},
           {"fhp", "First Hit (Points)", CompositingType::FirstHitPoints}},
          0, InvalidationLevel::InvalidResources)
    , gradientComputation_(
          "gradientComputationMode", "Gradient",
          {{"none", "None", GradientComputation::None},
           {"forward", "Forward Differences", GradientComputation::Forward},
           {"backward", "Backward Differences", GradientComputation::Backward},
           {"central", "Central Differences", GradientComputation::Central},
           {"central-higher", "Higher-order Central Differences",
            GradientComputation::CentralHigherOrder},
           {"precomputedXYZ", "Pre-computed Gradients (xyz)", GradientComputation::PrecomputedXYZ},
           {"precomputedYZW", "Pre-computed Gradients (yzw)", GradientComputation::PrecomputedYZW}},
          3, InvalidationLevel::InvalidResources)
    , samplingRate_("samplingRate", "Sampling rate", 2.0f, 1.0f, 20.0f) {

    addProperties(renderingType_, classification_, compositing_, gradientComputation_,
                  samplingRate_);
}

CustomRaycastingProperty::CustomRaycastingProperty(const CustomRaycastingProperty& rhs)
    : CompositeProperty(rhs)
    , renderingType_(rhs.renderingType_)
    , classification_(rhs.classification_)
    , compositing_(rhs.compositing_)
    , gradientComputation_(rhs.gradientComputation_)
    , samplingRate_(rhs.samplingRate_) {

    addProperties(renderingType_, classification_, compositing_, gradientComputation_,
                  samplingRate_);
}

CustomRaycastingProperty* CustomRaycastingProperty::clone() const { return new CustomRaycastingProperty(*this); }

} 