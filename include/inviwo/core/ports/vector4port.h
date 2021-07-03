#pragma once

#include <inviwo/core/common/inviwocoredefine.h>
#include <inviwo/core/ports/datainport.h>
#include <inviwo/core/ports/dataoutport.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/util/glm.h>


namespace inviwo {

/**
 * \ingroup ports
 */
using Vec4Inport = DataInport<std::vector<inviwo::vec4>>;

/**
 * \ingroup ports
 */
using Vec4Outport = DataOutport<std::vector<inviwo::vec4>>;

}  // namespace inviwo
