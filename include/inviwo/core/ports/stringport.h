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
using StringInport = DataInport<std::vector<std::string>>;

/**
 * \ingroup ports
 */
using StringOutport = DataOutport<std::vector<std::string>>;

}  // namespace inviwo
