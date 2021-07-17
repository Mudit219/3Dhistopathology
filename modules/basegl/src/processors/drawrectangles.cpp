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

#include <modules/basegl/processors/drawrectangles.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/opengl/openglutils.h>
#include <inviwo/core/datastructures/buffer/bufferramprecision.h>
#include <inviwo/core/interaction/events/keyboardevent.h>
#include <inviwo/core/interaction/events/mouseevent.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo DrawRectangles::processorInfo_{
    "org.inviwo.DrawRectangles",      // Class identifier
    "Draw Rectangles",                // Display name
    "Drawing",              // Category
    CodeState::Experimental,  // Code state
    Tags::GL,               // Tags
};
const ProcessorInfo DrawRectangles::getProcessorInfo() const { return processorInfo_; }

DrawRectangles::DrawRectangles()
    : Processor()
    , inport_("inport")
    , outport_("outport")
    , coordinateport_("coordinatePort")
    , lineSize_("lineSize", "Line Size", 1.f, 1.f, 10.f)
    , lineColor_("lineColor", "Line Color", vec4(1.f))
    , clearButton_("clearButton", "Clear Lines")
    , mouseDraw_(
          "mouseDraw", "Draw Line", [this](Event* e) { eventDraw(e); }, MouseButton::Left,
          MouseState::Press, KeyModifiers(flags::none))
    , keyEnableDraw_(
          "keyEnableDraw", "Enable Draw", [this](Event* e) { eventEnableDraw(e); }, IvwKey::Up,
          KeyStates(flags::any), KeyModifiers(flags::none))

    , lines_(DrawType::Triangles, ConnectivityType::Fan)
    , lineDrawer_(&lines_)
    , lineShader_("img_color.frag")
    , pointsCount_(0) {

    addPort(inport_);
    addPort(outport_);
    addPort(coordinateport_);

    addProperty(lineSize_);
    lineColor_.setSemantics(PropertySemantics::Color);
    addProperty(lineColor_);
    clearButton_.onChange([this]() { clearLines(); });
    addProperty(clearButton_);

    addProperty(mouseDraw_);
    addProperty(keyEnableDraw_);
    lineShader_.onReload([this]() { invalidate(InvalidationLevel::InvalidResources); });

    lines_.addBuffer(BufferType::PositionAttrib, std::make_shared<Buffer<vec2>>());

    GLint aliasRange[2];
    glGetIntegerv(GL_ALIASED_LINE_WIDTH_RANGE, aliasRange);

    lineSize_.setMinValue(static_cast<float>(aliasRange[0]));
    lineSize_.setMaxValue(static_cast<float>(aliasRange[1]));

    if (aliasRange[0] == aliasRange[1]) lineSize_.setVisible(false);
}

DrawRectangles::~DrawRectangles() = default;

void DrawRectangles::process() {
    utilgl::activateTargetAndCopySource(outport_, inport_, ImageType::ColorOnly);
    {
        utilgl::GlBoolState linesmooth(GL_LINE_SMOOTH, false);

        lineShader_.activate();
        lineShader_.setUniform("color", lineColor_);
        lineDrawer_.draw();
        lineShader_.deactivate();
    }
    utilgl::deactivateCurrentTarget();
    if(pointsCount_ == 4)
    {
        auto Xval = [](dvec2 a, dvec2 b)
        {
            return a[0] < b[0];
        };
        auto Yval = [](dvec2 a, dvec2 b)
        {
            return a[1] < b[1];
        };
        
        auto maxX_elm = *(std::max_element(corner_.begin(), corner_.end(), Xval));
        auto minX_elm = *(std::min_element(corner_.begin(), corner_.end(), Xval));
        auto maxY_elm = *(std::max_element(corner_.begin(), corner_.end(), Yval));
        auto minY_elm = *(std::min_element(corner_.begin(), corner_.end(), Yval));

        std::vector<inviwo::vec2> recVals = {vec2(minX_elm[0], minY_elm[1]), vec2(maxX_elm[0], minY_elm[1]), 
                                             vec2(minX_elm[0], maxY_elm[1]), vec2(maxX_elm[0], maxY_elm[1])};
        coordinateport_.setData(std::move(recVals));
    }
    
}


void DrawRectangles::addPoint(vec2 p, dvec2 o) {
    auto buff =
        static_cast<Vec2BufferRAM*>(lines_.getBuffer(0)->getEditableRepresentation<BufferRAM>());
    buff->add(p);
    corner_.push_back(o);
}

void DrawRectangles::clearLines() {
    auto buff =
        static_cast<Vec2BufferRAM*>(lines_.getBuffer(0)->getEditableRepresentation<BufferRAM>());

    buff->clear();
    corner_.clear();
}

void DrawRectangles::eventDraw(Event* event) {
    if (!drawModeEnabled_) return;

    auto mouseEvent = static_cast<MouseEvent*>(event);
    auto line = mouseEvent->ndc();
    auto origLine = mouseEvent->pos(); //get the original coordinates 

    pointsCount_++;
    if(pointsCount_>4)
    {
        clearLines();
        addPoint(vec2(line.x,line.y), origLine);
        pointsCount_ = 1;
    }
    else
    {
        addPoint(vec2(line.x, line.y), origLine);
    }
    invalidate(InvalidationLevel::InvalidOutput);
}

void DrawRectangles::eventEnableDraw(Event* event) {
    auto keyEvent = static_cast<KeyboardEvent*>(event);
    drawModeEnabled_ = (keyEvent->state() != KeyState::Release);
}

}  // namespace inviwo
