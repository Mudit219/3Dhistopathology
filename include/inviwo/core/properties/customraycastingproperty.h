#pragma once

#include <inviwo/core/common/inviwocoredefine.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/properties/compositeproperty.h>
#include <inviwo/core/properties/optionproperty.h>

namespace inviwo {

/**
 * \ingroup properties
 * \class CustomRaycastingProperty
 * \brief composite property holding parameters for volume raycasting
 */
class IVW_CORE_API CustomRaycastingProperty : public CompositeProperty {
public:
    virtual std::string getClassIdentifier() const override;
    static const std::string classIdentifier;

    enum class RenderingType { Dvr };
    enum class Classification { None, TF, Voxel };
    enum class CompositingType {
        Dvr,
        MaximumIntensity,
        FirstHitPoints,
        FirstHitNormals,
        FirstHistNormalsView,
        FirstHitDepth
    };
    enum class GradientComputation {
        None,
        Forward,
        Backward,
        Central,
        CentralHigherOrder,
        PrecomputedXYZ,
        PrecomputedYZW
    };

    CustomRaycastingProperty(std::string identifier, std::string displayName,
                       InvalidationLevel = InvalidationLevel::InvalidResources,
                       PropertySemantics semantics = PropertySemantics::Default);

    CustomRaycastingProperty(const CustomRaycastingProperty& rhs);
    virtual ~CustomRaycastingProperty() = default;

    virtual CustomRaycastingProperty* clone() const override;

    TemplateOptionProperty<RenderingType> renderingType_;
    TemplateOptionProperty<Classification> classification_;
    TemplateOptionProperty<CompositingType> compositing_;
    TemplateOptionProperty<GradientComputation> gradientComputation_;

    FloatProperty samplingRate_;
};
}