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


#include <modules/base/processors/custompixelvalue.h>
#include <inviwo/core/datastructures/image/layerram.h>
#include <inviwo/core/datastructures/image/layerramprecision.h>
#include <inviwo/core/datastructures/image/layerutil.h>
#include <inviwo/core/interaction/events/mouseevent.h>
#include <inviwo/core/util/indexmapper.h>
#include <inviwo/core/util/utilities.h>

namespace inviwo {

auto areaPixelsNormalized_ = std::make_shared<std::vector<inviwo::vec4>>();  //Will store the set of pixel colors. 
        
// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo CustomPixelValue::processorInfo_{
    "org.inviwo.CustomPixelValue",      // Class identifier
    "Custom Pixel Value",                // Display name
    "Information",              // Category
    CodeState::Stable,  // Code state
    Tags::CPU,               // Tags
};
const ProcessorInfo CustomPixelValue::getProcessorInfo() const { return processorInfo_; }

CustomPixelValue::CustomPixelValue()
    : Processor()
    , inport_("inport", true)
    , outport_("outport", false)
    , vecOutport_("outport2")
    , coordinates_("Coordinates", "Coordinates", size2_t(0),
                   size2_t(std::numeric_limits<size_t>::lowest()),
                   size2_t(std::numeric_limits<size_t>::max()), size2_t(1),
                   InvalidationLevel::Valid, PropertySemantics::Text)
    , imagedims_("Dimesions", "Image Dimensions", size2_t(0),
                  size2_t(std::numeric_limits<size_t>::lowest()),
                  size2_t(std::numeric_limits<size_t>::max()), size2_t(1),
                  InvalidationLevel::Valid, PropertySemantics::Text)
    , pixelValuesNormalized_({
          {"pixelValue1Normalized", "Normalized Pixel Value", vec4(0), vec4(0), vec4(1),
           vec4(std::numeric_limits<float>::epsilon()), InvalidationLevel::Valid,
           PropertySemantics::Color},
          {"pixelValue2Normalized", "Normalized Pixel Value (Layer 2)", vec4(0), vec4(0), vec4(1),
           vec4(std::numeric_limits<float>::epsilon()), InvalidationLevel::Valid,
           PropertySemantics::Color},
          {"pixelValue3Normalized", "Normalized Pixel Value (Layer 3)", vec4(0), vec4(0), vec4(1),
           vec4(std::numeric_limits<float>::epsilon()), InvalidationLevel::Valid,
           PropertySemantics::Color},
          {"pixelValue4Normalized", "Normalized Pixel Value (Layer 4)", vec4(0), vec4(0), vec4(1),
           vec4(std::numeric_limits<float>::epsilon()), InvalidationLevel::Valid,
           PropertySemantics::Color},
          {"pixelValue5Normalized", "Normalized Pixel Value (Layer 5)", vec4(0), vec4(0), vec4(1),
           vec4(std::numeric_limits<float>::epsilon()), InvalidationLevel::Valid,
           PropertySemantics::Color},
          {"pixelValue6Normalized", "Normalized Pixel Value (Layer 6)", vec4(0), vec4(0), vec4(1),
           vec4(std::numeric_limits<float>::epsilon()), InvalidationLevel::Valid,
           PropertySemantics::Color},
          {"pixelValue7Normalized", "Normalized Pixel Value (Layer 7)", vec4(0), vec4(0), vec4(1),
           vec4(std::numeric_limits<float>::epsilon()), InvalidationLevel::Valid,
           PropertySemantics::Color},
          {"pixelValue8Normalized", "Normalized Pixel Value (Layer 8)", vec4(0), vec4(0), vec4(1),
           vec4(std::numeric_limits<float>::epsilon()), InvalidationLevel::Valid,
           PropertySemantics::Color},

      })
    , mouseClick_(
          "mouseClick", "Mouse Click", [this](Event* e) { mouseClickEvent(e); },
          MouseButton::Left, MouseState::Press, KeyModifiers(flags::none),
          InvalidationLevel::InvalidResources,PropertySemantics::Default)
    , areaPixelsData_({inviwo::vec4(0.0,0.0,0.0,0.0)})
    , selectedPixelsData_(
        "selectedPixelsDataproperty", "Selected Pixels Property",
        std::make_unique<FloatVec4Property>(
            "Color1", "Color 1", vec4(0), vec4(0), vec4(1),
            vec4(std::numeric_limits<float>::epsilon()), 
            InvalidationLevel::InvalidResources, PropertySemantics::Color
            ),100
    )
    , MaxColorVal_(0)
    , threshold_("threshold", "Threshold", 2, 0, 10)

{
    addPort(inport_);
    addPort(outport_);
    addPort(vecOutport_);

    for (int i = 0; i < 8; i++) {
        pixelValuesNormalized_[i].setVisible(i == 0);
        addProperty(pixelValuesNormalized_[i]);
    }

    addProperty(coordinates_);
    addProperty(imagedims_);
    addProperty(mouseClick_);
    addProperty(selectedPixelsData_);
    addProperty(threshold_);

    inport_.onChange([&]() {
        size_t numCh = 1;
        if (inport_.hasData()) {
            numCh = inport_.getData()->getNumberOfColorLayers();
            imagedims_.set(inport_.getData()->getDimensions());
        }
        for (size_t i = 0; i < 8; i++) {
            pixelValuesNormalized_[i].setVisible(i < numCh);
        }
    });

    for (auto& p : getProperties()) {
        p->setSerializationMode(PropertySerializationMode::None);
    }

    selectedPixelsData_.PropertyOwnerObservable::addObserver(this);
}

void CustomPixelValue::process() 
{ 
    outport_.setData(inport_.getData());
    vecOutport_.setData(std::move(areaPixelsData_)); 
}

void CustomPixelValue::getRegionColors(size2_t pos )
{
    auto img = inport_.getData();
    auto dims = img->getDimensions();
    auto numCh = img->getNumberOfColorLayers();

    for (size_t i = 0; i < numCh; i++) {
            img->getColorLayer(i)
                ->getRepresentation<LayerRAM>()
                ->dispatch<void, dispatching::filter::All>([&](const auto layer) {
                    using ValueType = util::PrecisionValueType<decltype(layer)>;
                    using Comp = typename util::value_type<ValueType>::type;
                    const auto data = layer->getDataTyped();
                    const auto im = util::IndexMapper2D(dims);

                    auto value = data[im(pos)];
                    auto v = util::glm_convert<glm::vec<4, Comp>>(value);
                    v = util::applySwizzleMask(v, img->getColorLayer(i)->getSwizzleMask());

                    auto vf = util::glm_convert<glm::vec<4, float>>(v);
                    if constexpr (std::is_integral_v<Comp>) {
                        vf /= std::numeric_limits<Comp>::max();
                    }
                    std::vector<inviwo::vec4> selectedColorVal;
                    for(auto p : selectedPixelsData_.getProperties())
                    {
                        auto t = static_cast<FloatVec4Property*>(p);
                        selectedColorVal.push_back(t->get());
                    }
                    if(std::find(selectedColorVal.begin(), selectedColorVal.end(), vf) == selectedColorVal.end())
                    {
                        std::string selectedPixelsIdentifier = "Color" + std::to_string(MaxColorVal_+1);
                        std::string selectedPixelsName = "Color " + std::to_string(MaxColorVal_+1); 
                        selectedPixelsData_.addProperty(new FloatVec4Property(selectedPixelsIdentifier, selectedPixelsName,
                                                        vf,vec4(0), vec4(1),
                                                        vec4(std::numeric_limits<float>::epsilon()), 
                                                        InvalidationLevel::Valid,PropertySemantics::Color));
                        MaxColorVal_++;
                    }
                    areaPixelsData_ = selectedColorVal;
                });
        }
    return;
}

void CustomPixelValue::mouseClickEvent(Event* theevent) {
    if (!inport_.hasData()) return;

    areaPixelsData_.clear();

    if (auto mouseEvent = theevent->getAs<MouseEvent>()) {
        auto img = inport_.getData();
        auto dims = img->getDimensions();
        auto numCh = img->getNumberOfColorLayers();
        auto p = mouseEvent->posNormalized();
        if (glm::any(glm::lessThan(p, dvec2(0, 0))) || glm::any(glm::greaterThan(p, dvec2(1, 1)))) {
             return;
        }
        const size2_t pos{p * dvec2(dims - size2_t(1))};
        coordinates_.set(pos);

        for (size_t i = 0; i < numCh; i++) 
        {
            img->getColorLayer(i)
                ->getRepresentation<LayerRAM>()
                ->dispatch<void, dispatching::filter::All>([&](const auto layer) {
                    using ValueType = util::PrecisionValueType<decltype(layer)>;
                    using Comp = typename util::value_type<ValueType>::type;
                    const auto data = layer->getDataTyped();
                    const auto im = util::IndexMapper2D(dims);

                    auto value = data[im(pos)];
                    
                    auto v = util::glm_convert<glm::vec<4, Comp>>(value);
                    v = util::applySwizzleMask(v, img->getColorLayer(i)->getSwizzleMask());

                    auto vf = util::glm_convert<glm::vec<4, float>>(v);
                    if constexpr (std::is_integral_v<Comp>) {
                        vf /= std::numeric_limits<Comp>::max();
                    }
                    pixelValuesNormalized_[i].set(vf);
                    
                    //adding this color to the list

                    std::vector<inviwo::vec4> selectedColorVal;
                    for(auto p : selectedPixelsData_.getProperties())
                    {
                        auto t = static_cast<FloatVec4Property*>(p);
                        selectedColorVal.push_back(t->get());
                    }
                    if(std::find(selectedColorVal.begin(), selectedColorVal.end(), vf) == selectedColorVal.end())
                    {
                        std::string selectedPixelsIdentifier = "Color" + std::to_string(MaxColorVal_+1);
                        std::string selectedPixelsName = "Color " + std::to_string(MaxColorVal_+1); 
                        selectedPixelsData_.addProperty(new FloatVec4Property(selectedPixelsIdentifier, selectedPixelsName,
                                                        vf,vec4(0), vec4(1),
                                                        vec4(std::numeric_limits<float>::epsilon()), 
                                                        InvalidationLevel::Valid,PropertySemantics::Color));
                        MaxColorVal_++;
                    }
                    areaPixelsData_ = selectedColorVal;
                });
        }

        for(long unsigned int i=0; i < threshold_.get();i++)
        {
            for(long unsigned int j=0; j< threshold_.get(); j++)
            {
                auto a = coordinates_.get();
                a[0] += i;
                a[1] += j;
                const size2_t a_pos{a};
                getRegionColors(a_pos);

                if(i == 0 && j == 0)continue;

                auto b = coordinates_.get();
                b[0] += i;
                b[1] -= j;
                const size2_t b_pos{b};
                getRegionColors(b_pos);

                auto c = coordinates_.get();
                c[0] -= i;
                c[1] += j;
                const size2_t c_pos{c};
                getRegionColors(c_pos);

                auto d = coordinates_.get();
                d[0] -= i;
                d[1] -= j;
                const size2_t d_pos{d};
                getRegionColors(d_pos);

            }
        }
    }
}

void CustomPixelValue::onDidAddProperty(Property* property, size_t) {
    updateOptions();
    property->Property::addObserver(this);
}

void CustomPixelValue::onDidRemoveProperty(Property* property, size_t) 
{ 
    updateOptions(); 
    property->Property::removeObserver(this);
}

void CustomPixelValue::updateOptions() 
{
    std::vector<inviwo::vec4> selectedColorVal;
    for(auto p : selectedPixelsData_.getProperties())
    {
        auto t = static_cast<FloatVec4Property*>(p);
        selectedColorVal.push_back(t->get());
    }
    areaPixelsData_ = selectedColorVal;
    vecOutport_.setData(std::move(areaPixelsData_)); 
}

}  // namespace inviwo
