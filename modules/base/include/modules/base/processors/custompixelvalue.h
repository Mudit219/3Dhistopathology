/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2021 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#ifndef IVW_CUSTOMPIXELVALUE_H
#define IVW_CUSTOMPIXELVALUE_H



#include <modules/base/basemoduledefine.h>
#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/properties/listproperty.h>
#include <inviwo/core/properties/stringproperty.h>
#include <inviwo/core/properties/eventproperty.h>
#include <inviwo/core/properties/propertyownerobserver.h>
#include <inviwo/core/ports/imageport.h>
#include <inviwo/core/ports/vector4port.h>

namespace inviwo {

/** \docpage{org.inviwo.CustomPixelValue, Custom Pixel Value}
 * ![](org.inviwo.CustomPixelValue.png?classIdentifier=org.inviwo.CustomPixelValue)
 * Read the pixel value under the mouse of the image that is passed through the processor
 *
 * ### Inport
 *   * __inport___ Input image
 *
 * ### Outport
 *   * __outport__ Output image, pass through of input image
 *
 * ### Properties
 *   * __Pixel Value__ The pixel value under the mouse of the first color layer
 *   * __Pixel Value (as string)__ As a string.
 *   * __Normalized Pixel Value__ Normalized to [0,1]
 *   * __Picking Value__ The pixel value under the mouse of the picking layer
 *   * __Picking Value (as string)__ As a string.
 *   * __Depth Value__ The depth value under the mouse of the depth layer
 *   * __Depth Value (as string)__ As a string.
 *   * __Coordinates__ The mouse coordinates in the image.
 *
 */
class IVW_MODULE_BASE_API CustomPixelValue : public Processor,
                                             public PropertyObserver,
                                             public PropertyOwnerObserver {
public:
    CustomPixelValue();
    virtual ~CustomPixelValue() = default;

    virtual void process() override;

    void mouseClickEvent(Event* theevent);

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

    void getRegionColors(size2_t);

private:
    
    virtual void onDidAddProperty(Property* property, size_t index) override;
    virtual void onDidRemoveProperty(Property* property, size_t index) override;

    void updateOptions();

    ImageInport inport_;
    ImageOutport outport_;
    Vec4Outport vecOutport_;

    IntSize2Property coordinates_;
    // std::vector<DoubleVec4Property> pixelValues_;
    std::vector<FloatVec4Property> pixelValuesNormalized_;
    // DoubleVec4Property pickingValue_;
    // DoubleProperty depthValue_;

    // std::vector<StringProperty> pixelStrValues_;
    // StringProperty pickingStrValue_;
    // StringProperty depthStrValue_;

    EventProperty mouseClick_;
    std::vector<inviwo::vec4> areaPixelsData_;

    ListProperty selectedPixelsData_;
    int MaxColorVal_;
    IntSizeTProperty threshold_;
};

}  // namespace inviwo

#endif  // IVW_CUSTOMPIXELVALUE_H