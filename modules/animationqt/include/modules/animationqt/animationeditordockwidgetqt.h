/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2016-2021 Inviwo Foundation
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
#pragma once

#include <modules/animationqt/animationqtmoduledefine.h>
#include <inviwo/core/common/inviwo.h>

#include <modules/qtwidgets/inviwodockwidget.h>

#include <modules/animation/animationcontrollerobserver.h>

class QToolButton;
class QMainWindow;
class QComboBox;

namespace inviwo {

namespace animation {

class AnimationManager;
class AnimationController;
class AnimationEditorQt;
class AnimationViewQt;
class SequenceEditorPanel;
class TrackWidgetQtFactory;
class SequenceEditorFactory;
class WorkspaceAnimations;

class IVW_MODULE_ANIMATIONQT_API AnimationEditorDockWidgetQt : public InviwoDockWidget,
                                                               public AnimationControllerObserver {
public:
    AnimationEditorDockWidgetQt(WorkspaceAnimations& animations, AnimationManager& manager,
                                const std::string& widgetName, TrackWidgetQtFactory& widgetFactory,
                                SequenceEditorFactory& editorFactory, QWidget* parent);
    AnimationEditorDockWidgetQt(const AnimationEditorDockWidgetQt&) = delete;
    AnimationEditorDockWidgetQt(AnimationEditorDockWidgetQt&&) = delete;
    AnimationEditorDockWidgetQt& operator=(const AnimationEditorDockWidgetQt&) = delete;
    AnimationEditorDockWidgetQt& operator=(AnimationEditorDockWidgetQt&&) = delete;
    virtual ~AnimationEditorDockWidgetQt();

    /**
     * Shows a file dialog for loading Animations from an Inviwo workspace and imports them.
     */
    void importAnimation();

protected:
    virtual void onStateChanged(AnimationController* controller, AnimationState prevState,
                                AnimationState newState) override;

    // Selected Animation in animationsList_ changed
    virtual void onAnimationChanged(AnimationController* controller, Animation* oldAnim,
                                    Animation* newAnim) override;

    WorkspaceAnimations& animations_;
    AnimationController& controller_;
    AnimationManager& manager_;

    // GUI-stuff
    QComboBox* animationsList_;
    QAction* btnPlayPause_;
    std::unique_ptr<AnimationEditorQt> animationEditor_;
    AnimationViewQt* animationView_;
    SequenceEditorPanel* sequenceEditorView_;
    QMainWindow* mainWindow_;
    bool vScrolling_ = false;
};

}  // namespace animation

}  // namespace inviwo
