/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2019-2021 Inviwo Foundation
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

#include <modules/base/pythonbindings/io/volumewriting.h>

#include <modules/base/io/datvolumewriter.h>
#include <modules/base/io/ivfvolumewriter.h>
#include <modules/base/io/ivfsequencevolumewriter.h>

#include <warn/push>
#include <warn/ignore/shadow>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <warn/pop>

namespace inviwo {

void exposeVolumeWriteMethods(pybind11::module& m) {

    m.def("saveDatVolume", &util::writeDatVolume);
    m.def("saveIvfVolume", &util::writeIvfVolume);
    m.def("saveIvfVolumeSequence", &util::writeIvfVolumeSequence);
    m.def("saveIvfVolumeSequence", [](pybind11::list list, std::string name, std::string path,
                                      std::string reltivePathToTimesteps, bool overwrite) {
        VolumeSequence seq;
        for (auto&& v : list) {
            seq.push_back(v.cast<std::shared_ptr<Volume>>());
        }

        return util::writeIvfVolumeSequence(seq, name, path, reltivePathToTimesteps, overwrite);
    });
}

}  // namespace inviwo
