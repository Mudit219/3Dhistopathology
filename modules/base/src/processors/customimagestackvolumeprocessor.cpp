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

#include <modules/base/processors/customimagestackvolumeprocessor.h>

#include <inviwo/core/common/inviwoapplication.h>
#include <inviwo/core/datastructures/image/layer.h>
#include <inviwo/core/datastructures/image/layerram.h>
#include <inviwo/core/datastructures/image/layerramprecision.h>
#include <inviwo/core/datastructures/image/imageram.h>
#include <inviwo/core/datastructures/volume/volume.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>
#include <inviwo/core/io/datareaderfactory.h>
#include <inviwo/core/util/filesystem.h>
#include <inviwo/core/util/stdextensions.h>
#include <inviwo/core/util/vectoroperations.h>
#include <inviwo/core/util/zip.h>
#include <inviwo/core/util/raiiutils.h>
#include <inviwo/core/io/datareaderexception.h>

#include <algorithm>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include <openslide/openslide.h>
#include<fstream>
namespace inviwo {

namespace {

template <typename Format>
struct FloatOrIntMax32
    : std::integral_constant<bool, Format::numtype == NumericType::Float || Format::compsize <= 4> {
};

}  // namespace


// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo CustomImageStackVolumeProcessor::processorInfo_{
    "org.inviwo.CustomImageStackVolumeProcessor",      // Class identifier
    "CustomImageStackVolumeProcessor",                // Display name
    "Undefined",              // Category
    CodeState::Experimental,  // Code state
    Tags::None,               // Tags
};
const ProcessorInfo CustomImageStackVolumeProcessor::getProcessorInfo() const { return processorInfo_; }

CustomImageStackVolumeProcessor::CustomImageStackVolumeProcessor(InviwoApplication* app)
    : Processor()
    , inport_("RegionCoordinates")
    , outport_("volume")
    , filePattern_("filePattern", "File Pattern", "####.jpeg", "")
    , reload_("reload", "Reload data")
    , skipUnsupportedFiles_("skipUnsupportedFiles", "Skip Unsupported Files", false)
    , basis_("Basis", "Basis and offset")
    , information_("Information", "Data information")
    , readerFactory_{app->getDataReaderFactory()}
    {
    addPort(inport_);
    addPort(outport_);
    addProperty(filePattern_);
    addProperty(reload_);
    addProperty(skipUnsupportedFiles_);
    addProperty(basis_);
    addProperty(information_);
    // myFile.open("/home/sassluck/Desktop/Histopathology/level2.jpg", std::ios_base::out | std::ios_base::binary);
    isSink_.setUpdate([]() { return true; });
    isReady_.setUpdate([this]() { return !filePattern_.getFileList().empty(); });
    filePattern_.onChange([&]() { isReady_.update(); });

    addFileNameFilters();
}
std::ofstream myFile;
void CustomImageStackVolumeProcessor::myOutput(unsigned char byte)
{
  myFile << byte;
}
void CustomImageStackVolumeProcessor::my_slide(std::string PATH){
        std::cout << "Image path is : "<< PATH << std::endl;

    openslide_t* op = openslide_open(PATH.c_str());
    int32_t level = 2;
    int64_t dim_lvlk[2];
    int64_t dim_lvl0[2];
    if(op!=0)
    {
        openslide_get_level_dimensions(op,0,&dim_lvl0[0],&dim_lvl0[1]);
        openslide_get_level_dimensions(op,level,&dim_lvlk[0],&dim_lvlk[1]);
    }
    else
        std::cout << "Please enter a valid image path" << std::endl;
    /*
    Rectangle based calculation 
    */
    int64_t start_x,start_y,w,h;
    // int64_t size_img=w*h*4;
    // uint32_t *dest = (uint32_t*)malloc(size_img);
    if(!inport_.hasData())
    {
        std::cout << "No" << std::endl;
        start_x=0,start_y=0,w=dim_lvlk[0]/2,h=dim_lvlk[1]/2; 
    }
    else
    {
        std::cout << "Yes" << std::endl;
        start_x = inport_.getData()->at(0).x;
        start_y = inport_.getData()->at(0).y;

        w = abs(inport_.getData()->at(0).x - inport_.getData()->at(1).x);
        h = abs(inport_.getData()->at(0).y - inport_.getData()->at(2).y);

        std::cout << start_x << " " << start_y << " " << w << " " << h << std::endl; 
    }
    start_x*=(dim_lvl0[0]/dim_lvlk[0]);
    start_y*=(dim_lvl0[1]/dim_lvlk[1]);

    int64_t size_img=w*h*4;
    uint32_t *dest = (uint32_t*)malloc(size_img);
    

    if(op!=0){
        openslide_read_region(op,dest,start_x,start_y,level,w,h);
        openslide_close(op);
    }
    // std::cout << openslide_get_level_count(op);
    // std::cout << w << "," <<  h;
    // std::cout << *dest;
    // std::cout << sizeof(dest)/sizeof(dest[0]);
    std::cout<< "(" << dim_lvlk[0] << "," << dim_lvlk[1] << ")" << std::endl;
    // std::cout << "size : " <<size_img << std::endl; 
    // convert_to_binary(65523);
    // color
    // for(int i=0;i<size_img;i++)
    //     std::cout << dest[i] <<  " ";
    int bytesperpixel=3;
    auto pixels = new unsigned char[w*h*bytesperpixel];
    // std::cout << "Total number of pixels :" << w*h*bytesperpixel << std::endl;

    for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
        uint32_t argb = dest[y * w + x];
        // double alpha = (argb >> 24)& 0xff;
        double red = (argb >> 16)& 0xff;
        double green = (argb >> 8)& 0xff;
        double blue = (argb >> 0)& 0xff;
        // std::cout << red << " " << green << " " << blue << std::endl;
        int offset = (y * w + x)*bytesperpixel;
        pixels[offset]=red;
        pixels[offset+1]=green;
        pixels[offset+2]=blue;
        // std::cout << alpha;
    //   std::cout << ((argb >> 16)& 0xff) << " ";
    }}
    // free(dest);
    // delete []pixels;
    TooJpeg::writeJpeg(myOutput, pixels, w, h);
    std::ofstream out("/home/sassluck/Desktop/Histopathology/output.txt");
    std::string img_name = "/home/sassluck/Desktop/Histopathology/level2.jpg\n";
    for(int i=0;i<10;i++)
        out << img_name;
    out.close();
    // std::cout << "Openslide is working" << std::endl;
}

void CustomImageStackVolumeProcessor::addFileNameFilters() {
    filePattern_.clearNameFilters();
    filePattern_.addNameFilter(FileExtension::all());
    filePattern_.addNameFilters(readerFactory_->getExtensionsForType<Layer>());
}

void CustomImageStackVolumeProcessor::process() {
    // outport_.setData(myImage);
    // my_slide();
    util::OnScopeExit guard{[&]() { outport_.setData(nullptr); }};
    myFile.open("/home/sassluck/Desktop/Histopathology/level2.jpg", std::ios_base::out | std::ios_base::binary);
    myFile.close();
    if (filePattern_.isModified() || reload_.isModified() || skipUnsupportedFiles_.isModified()) {
        myFile.open("/home/sassluck/Desktop/Histopathology/level2.jpg", std::ios_base::out | std::ios_base::binary);
        auto image_paths = filePattern_.getFileList();
        std::cout << "Current image path is : " << image_paths[0] << std::endl;
        my_slide(image_paths[0]);
        volume_ = load();
        if (volume_) {
            basis_.updateForNewEntity(*volume_, deserialized_);
            information_.updateForNewVolume(*volume_, deserialized_);
        }
        deserialized_ = false;
    }

    if (volume_) {
        basis_.updateEntity(*volume_);
        information_.updateVolume(*volume_);
    }
    outport_.setData(volume_);
    guard.release();
}

bool CustomImageStackVolumeProcessor::isValidImageFile(std::string fileName) {
    return readerFactory_->hasReaderForTypeAndExtension<Layer>(fileName);
}

std::shared_ptr<Volume> CustomImageStackVolumeProcessor::load() {
    std::ifstream file1("/home/sassluck/Desktop/Histopathology/output.txt");
    std::string img;
    std::vector<std::string> img_names;
    while(getline(file1,img)){
        img_names.push_back(img);
    }
    const auto files = img_names;
    for(uint64_t i=0;i<files.size();i++)
        std::cout << files[i] << std::endl;
    // std::cout << "This is my file size : " << files.size() << endl;
    // std::cout << files
    if (files.empty()) {
        return nullptr;
    }

    std::vector<std::pair<std::string, std::unique_ptr<DataReaderType<Layer>>>> slices;
    slices.reserve(files.size());

    std::transform(
        files.begin(), files.end(), std::back_inserter(slices),
        [&](const auto& file) -> std::pair<std::string, std::unique_ptr<DataReaderType<Layer>>> {
            return {file, std::move(readerFactory_->getReaderForTypeAndExtension<Layer>(
                              filePattern_.getSelectedExtension(), file))};
        });
    if (skipUnsupportedFiles_) {
        slices.erase(std::remove_if(slices.begin(), slices.end(),
                                    [](auto& elem) { return elem.second == nullptr; }),
                     slices.end());
    }

    // identify first slice with a reader
    const auto first = std::find_if(slices.begin(), slices.end(),
                                    [](auto& item) { return item.second != nullptr; });
    if (first == slices.end()) {  // could not find any suitable data reader for the images
        throw Exception(
            fmt::format("No supported images found in '{}'", filePattern_.getFilePatternPath()),
            IVW_CONTEXT);
    }

    const auto referenceLayer = first->second->readData(first->first);
    // std::cout << " This is the first layer : " << referenceLayer ;
    // Call getRepresentation here to enforce creating a ram representation.
    // Otherwise the default image size, i.e. 256x256, will be reported since the LayerDisk
    // does not provide meta data for all image formats.
    const auto referenceRAM = referenceLayer->getRepresentation<LayerRAM>();
    if (glm::compMul(referenceRAM->getDimensions()) == 0) {
        throw Exception(
            fmt::format("Could not extract valid image dimensions from '{}'", first->first),
            IVW_CONTEXT);
    }

    const auto refFormat = referenceRAM->getDataFormat();
    if ((refFormat->getNumericType() != NumericType::Float) && (refFormat->getPrecision() > 32)) {
        throw DataReaderException(
            fmt::format("Unsupported integer bit depth ({})", refFormat->getPrecision()),
            IVW_CONTEXT);
    }

    return referenceRAM->dispatch<std::shared_ptr<Volume>, FloatOrIntMax32>(
        [&](auto reflayerprecision) {
            using ValueType = util::PrecisionValueType<decltype(reflayerprecision)>;
            using PrimitiveType = typename DataFormat<ValueType>::primitive;

            const size2_t layerDims = reflayerprecision->getDimensions();
            const size_t sliceOffset = glm::compMul(layerDims);

            // create matching volume representation
            auto volumeRAM =
                std::make_shared<VolumeRAMPrecision<ValueType>>(size3_t{layerDims, slices.size()});
            auto volData = volumeRAM->getDataTyped();

            const auto fill = [&](size_t s) {
                std::fill(volData + s * sliceOffset, volData + (s + 1) * sliceOffset, ValueType{0});
            };

            const auto read = [&](const auto& file, auto reader) -> std::shared_ptr<Layer> {
                try {
                    return reader->readData(file);
                } catch (DataReaderException const& e) {
                    LogProcessorWarn(
                        fmt::format("Could not load image: {}, {}", file, e.getMessage()));
                    return nullptr;
                }
            };

            for (auto&& elem : util::enumerate(slices)) {
                const auto slice = elem.first();
                const auto file = elem.second().first;
                const auto reader = elem.second().second.get();
                if (!reader) {
                    fill(slice);
                    continue;
                }

                const auto layer = read(file, reader);
                if (!layer) {
                    fill(slice);
                    continue;
                }
                const auto layerRAM = layer->template getRepresentation<LayerRAM>();

                const auto format = layerRAM->getDataFormat();
                if ((format->getNumericType() != NumericType::Float) &&
                    (format->getPrecision() > 32)) {
                    LogProcessorWarn(fmt::format("Unsupported integer bit depth: {}, for image: {}",
                                                 format->getPrecision(), file));
                    fill(slice);
                    continue;
                }

                if (layerRAM->getDimensions() != layerDims) {
                    LogProcessorWarn(
                        fmt::format("Unexpected dimensions: {} , expected: {}, for image: {}",
                                    layer->getDimensions(), layerDims, file));
                    fill(slice);
                    continue;
                }
                layerRAM->template dispatch<void, FloatOrIntMax32>([&](auto layerpr) {
                    const auto data = layerpr->getDataTyped();
                    std::transform(
                        data, data + sliceOffset, volData + slice * sliceOffset,
                        [](auto value) { return util::glm_convert_normalized<ValueType>(value); });
                });
            }

            auto volume = std::make_shared<Volume>(volumeRAM);
            volume->dataMap_.dataRange =
                dvec2{DataFormat<PrimitiveType>::lowest(), DataFormat<PrimitiveType>::max()};
            volume->dataMap_.valueRange =
                dvec2{DataFormat<PrimitiveType>::lowest(), DataFormat<PrimitiveType>::max()};

            const auto size = vec3(0.01f) * static_cast<vec3>(volumeRAM->getDimensions());
            volume->setBasis(glm::diagonal3x3(size));
            volume->setOffset(-0.5 * size);

            return volume;
        });
}

void CustomImageStackVolumeProcessor::deserialize(Deserializer& d) {
    Processor::deserialize(d);
    addFileNameFilters();
    deserialized_ = true;
}

}  // namespace inviwo