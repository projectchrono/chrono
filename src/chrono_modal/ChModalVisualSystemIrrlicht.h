// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dario Mangoni
// =============================================================================
//
// Irrlicht-based visualization wrapper for modal. This class is a derived
// from ChVisualSystemIrrlicht and provides functionality to draw mode shapes
// =============================================================================

#ifndef CH_MODAL_VISUAL_SYSTEM_IRRLICHT_H
#define CH_MODAL_VISUAL_SYSTEM_IRRLICHT_H

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_modal/ChModalSolver.h"

namespace chrono {
namespace modal {

/// @addtogroup modal_vis
/// @{

template <typename ScalarType>
class ChModalVisualSystemIrrlicht;

template <typename ScalarType>
class ChModalEventReceiver : public irr::IEventReceiver {
  public:
    // Construct a custom event receiver.
    ChModalEventReceiver(ChModalVisualSystemIrrlicht<ScalarType>* vsys) : m_modal_vsys(vsys) {}

    virtual bool OnEvent(const irr::SEvent& event) {
        // Process GUI events
        if (event.EventType == irr::EET_GUI_EVENT) {
            irr::s32 id = event.GUIEvent.Caller->getID();

            switch (event.GUIEvent.EventType) {
                case irr::gui::EGET_EDITBOX_ENTER:
                    switch (id) {
                        case 9927: {
                            double val = atof(
                                irr::core::stringc(((irr::gui::IGUIEditBox*)event.GUIEvent.Caller)->getText()).c_str());
                            m_modal_vsys->SetAmplitude(val);

                        } break;
                        case 9928: {
                            double val = atof(
                                irr::core::stringc(((irr::gui::IGUIEditBox*)event.GUIEvent.Caller)->getText()).c_str());
                            m_modal_vsys->SetSpeedFactor(val);

                        } break;
                    }
                    break;

                case irr::gui::EGET_SCROLL_BAR_CHANGED:
                    switch (id) {
                        case 9926: {
                            int mode_sel = ((irr::gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                            m_modal_vsys->SetMode(mode_sel);
                        } break;
                    }
                    break;

                case irr::gui::EGET_CHECKBOX_CHANGED:
                    switch (id) {
                        case 9929: {
                            bool fix_freq = ((irr::gui::IGUICheckBox*)event.GUIEvent.Caller)->isChecked();
                            m_modal_vsys->SetFixedFrequency(fix_freq);
                        } break;
                    }
                    break;

                default:
                    break;
            }
        }

        return false;
    }

  private:
    ChModalVisualSystemIrrlicht<ScalarType>* m_modal_vsys;
};

template <typename ScalarType>
class ChModalVisualSystemIrrlicht : public irrlicht::ChVisualSystemIrrlicht {
  public:
    ChModalVisualSystemIrrlicht() {
        SetWindowSize(1024, 768);
        SetWindowTitle("Chrono::Modal");
    }

    virtual ~ChModalVisualSystemIrrlicht();

    /// Initialize the visualization system.
    virtual void Initialize() override;

    virtual void BeginScene(bool backBuffer, bool zBuffer, ChColor color) override;

    using irrlicht::ChVisualSystemIrrlicht::BeginScene;

    /// Render the Irrlicht scene and additional visual elements.
    virtual void Render() override;

    /// Set the mode to be visualized.
    /// The mode is specified as a 1-based index.
    void SetMode(int mode);

    /// Reset the timer used to generate the mode shape animation.
    void ResetTimer() { m_timer.reset(); }

    /// Set the amplitude of the mode shape animation.
    /// Eigenvectors are normalized and then multiplied by this amplitude.
    void SetAmplitude(double amplitude);

    /// Set the speed factor of the mode shape animation.
    /// The frequency is multiplied by this factor.
    void SetSpeedFactor(double speed_factor);

    /// Fix the frequency of the mode shape animation.
    /// The frequency is then fixed to 1 Hz.
    /// This is useful to visualize the mode shape without the frequency effect.
    /// The speed factor is still applied.
    void SetFixedFrequency(bool val);

    /// Reset the initial state by retrieving the current state of the assembly.
    void ResetInitialState();

    /// Attach the assembly to be visualized.
    /// This function must be called before starting the simulation loop.
    /// The eigenvectors and frequencies must be provided, typically through ChModalSolverUndamped or
    /// ChModalSolverDamped.
    void AttachAssembly(const ChAssembly& assembly,
                        const ChMatrixDynamic<ScalarType>& eigvects,
                        const ChVectorDynamic<double>& freq);

    /// Update the eigenvectors and frequencies.
    void UpdateModes(const ChMatrixDynamic<ScalarType>& eigvects, const ChVectorDynamic<double>& freq);

    /// Update the eigenvectors, frequencies, and damping ratios.
    template <typename U = ScalarType>
    typename std::enable_if<std::is_same<U, std::complex<double>>::value, void>::type UpdateModes(
        const ChMatrixDynamic<ScalarType>& eigvects,
        const ChVectorDynamic<double>& freq,
        const ChVectorDynamic<double>& damping_ratios);

  protected:
    ChAssembly* m_assembly = nullptr;
    const ChMatrixDynamic<ScalarType>* m_eigvects = nullptr;
    const ChVectorDynamic<double>* m_freq = nullptr;
    const ChVectorDynamic<double>* m_damping_ratios = nullptr;

    mutable int m_selected_mode = 0;  ///< currently selected mode
    ChTimer m_timer;                  ///< timer for mode shape animation

    ChState m_assembly_initial_state;  ///< initial state of the assembly
    double m_amplitude = 1.0;          ///< amplitude scaling of the mode shape animation
    double m_speed_factor = 1.0;       ///< speed factor of the mode shape animation
    bool m_fixed_frequency = false;    ///< fix the frequency of the animation

    /// Get the mode shape at a specified angle from an eigenvector (real eigvect).
    template <typename U = ScalarType>
    typename std::enable_if<std::is_same<U, double>::value, ChVectorDynamic<double>>::type GetModeShape(
        const ChVectorDynamic<double>& eigv,
        double angle) {
        return eigv.stableNormalized() * sin(angle);
    }

    /// Get the mode shape at a specified angle from an eigenvector (complex eigvect).
    template <typename U = ScalarType>
    typename std::enable_if<std::is_same<U, std::complex<double>>::value, ChVectorDynamic<double>>::type GetModeShape(
        const ChVectorDynamic<std::complex<double>>& eigv,
        double angle) {
        return eigv.stableNormalized().real() * sin(angle) + eigv.stableNormalized().imag() * cos(angle);
    }

    // UI components
    irr::gui::IGUIScrollBar* g_selected_mode;
    irr::gui::IGUIStaticText* g_selected_mode_info;
    irr::gui::IGUIEditBox* g_amplitude;
    irr::gui::IGUIEditBox* g_speed_factor;
    irr::gui::IGUICheckBox* g_fixed_frequency;
    ChModalEventReceiver<ScalarType>* m_receiver;
};

//// ChModalVisualSystemIrrlicht definitions ////

template <typename ScalarType>
ChModalVisualSystemIrrlicht<ScalarType>::~ChModalVisualSystemIrrlicht() {
    delete m_receiver;
}

template <typename ScalarType>
void ChModalVisualSystemIrrlicht<ScalarType>::Initialize() {
    bool m_modal_initialized = m_initialized;
    ChVisualSystemIrrlicht::Initialize();
    if (m_modal_initialized)
        return;

    ResetInitialState();

    // Set the modal event receiver
    m_receiver = new ChModalEventReceiver<ScalarType>(this);
    AddUserEventReceiver(m_receiver);

    auto guienv = GetDevice()->getGUIEnvironment();

    auto gui = GetGUI();

    auto g_tab2 = gui->AddTab(L"Modal");

    guienv->addStaticText(L"Amplitude", irr::core::rect<irr::s32>(10, 10, 100, 10 + 15), false, false, g_tab2);
    g_amplitude = guienv->addEditBox(L"", irr::core::rect<irr::s32>(100, 10, 140, 10 + 15), true, g_tab2, 9927);
    SetAmplitude(m_amplitude);

    guienv->addStaticText(L"Speed Factor", irr::core::rect<irr::s32>(10, 25, 100, 25 + 15), false, false, g_tab2);
    g_speed_factor = guienv->addEditBox(L"", irr::core::rect<irr::s32>(100, 25, 140, 25 + 15), true, g_tab2, 9928);
    SetSpeedFactor(m_speed_factor);

    guienv->addStaticText(L"Fix frequency", irr::core::rect<irr::s32>(10, 40, 100, 25 + 15 + 15), false, false, g_tab2);
    g_fixed_frequency = guienv->addCheckBox(m_fixed_frequency, irr::core::rect<irr::s32>(100, 40, 120, 25 + 15 + 15),
                                            g_tab2, 9929, L"Fix frequency");
    SetFixedFrequency(m_fixed_frequency);

    guienv->addStaticText(L"Mode", irr::core::rect<irr::s32>(10, 65, 100, 50 + 15 + 15), false, false, g_tab2);
    g_selected_mode =
        GetGUIEnvironment()->addScrollBar(true, irr::core::rect<irr::s32>(10, 80, 200, 65 + 15 + 15), g_tab2, 9926);
    g_selected_mode->setMin(1);
    g_selected_mode->setMax(1);
    g_selected_mode->setSmallStep(1);
    g_selected_mode->setLargeStep(10);
    g_selected_mode_info = GetGUIEnvironment()->addStaticText(
        L"", irr::core::rect<irr::s32>(10, 65 + 15 + 15, 200, 65 + 15 + 45 + 15), false, false, g_tab2);

    g_selected_mode->setEnabled(true);
    g_selected_mode_info->setEnabled(true);
    g_amplitude->setEnabled(true);
    g_speed_factor->setEnabled(true);
}

template <typename ScalarType>
inline void ChModalVisualSystemIrrlicht<ScalarType>::BeginScene(bool backBuffer, bool zBuffer, ChColor color) {
    ChVisualSystemIrrlicht::BeginScene(backBuffer, zBuffer, color);

    // check if the timer is running, if not, start it

    if (m_timer.GetTimeMilliseconds() == 0.0)
        m_timer.start();

    double angle =
        m_speed_factor * CH_2PI * (m_fixed_frequency ? 1.0 : m_freq->coeff(m_selected_mode)) * m_timer.GetTimeSeconds();

    ChState assembly_state_new;
    ChStateDelta assembly_state_delta;
    ChStateDelta assembly_v_dummy;

    double time_dummy = 0;

    assembly_state_new.setZero(m_assembly->GetNumCoordsPosLevel(), nullptr);
    assembly_state_delta.setZero(m_assembly->GetNumCoordsVelLevel(), nullptr);
    assembly_v_dummy.setZero(m_assembly->GetNumCoordsVelLevel(), nullptr);

    assembly_state_delta = m_amplitude * GetModeShape<>(m_eigvects->col(m_selected_mode), angle);

    m_assembly->IntStateIncrement(0, assembly_state_new, m_assembly_initial_state, 0, assembly_state_delta);
    m_assembly->IntStateScatter(0, assembly_state_new, 0, assembly_v_dummy, time_dummy, true);

    m_assembly->Update();

    OnUpdate(m_systems[0]);
}

template <typename ScalarType>
void ChModalVisualSystemIrrlicht<ScalarType>::Render() {
    ChVisualSystemIrrlicht::Render();
}

template <typename ScalarType>
void ChModalVisualSystemIrrlicht<ScalarType>::ResetInitialState() {
    if (!m_assembly)
        return;

    m_assembly->Setup();
    m_assembly_initial_state.setZero(m_assembly->GetNumCoordsPosLevel(), nullptr);

    ChStateDelta dummy_state_delta;
    dummy_state_delta.setZero(m_assembly->GetNumCoordsVelLevel(), nullptr);

    double time;
    m_assembly->IntStateGather(0, m_assembly_initial_state, 0, dummy_state_delta, time);

    m_timer.reset();
}

template <typename ScalarType>
void ChModalVisualSystemIrrlicht<ScalarType>::SetMode(int mode) {
    if (mode - 1 < 0 || mode - 1 >= (int)m_eigvects->cols()) {
        g_selected_mode->setPos(1);
        return;
    }

    m_selected_mode = mode - 1;

    g_selected_mode->setPos(m_selected_mode + 1);

    char message[50];
    if (m_damping_ratios)
        snprintf(message, sizeof(message), "n = %i\nf = %.3g Hz\nz = %.2g", m_selected_mode + 1,
                 (*m_freq)[m_selected_mode], (*m_damping_ratios)[m_selected_mode]);
    else
        snprintf(message, sizeof(message), "n = %i\nf = %.3g Hz", m_selected_mode + 1, (*m_freq)[m_selected_mode]);
    g_selected_mode_info->setText(irr::core::stringw(message).c_str());
}

template <typename ScalarType>
void ChModalVisualSystemIrrlicht<ScalarType>::SetAmplitude(double amplitude) {
    m_amplitude = std::max(0.0, amplitude);
    char message[50];
    snprintf(message, sizeof(message), "%g", m_amplitude);
    g_amplitude->setText(irr::core::stringw(message).c_str());
}

template <typename ScalarType>
void ChModalVisualSystemIrrlicht<ScalarType>::SetSpeedFactor(double speed_factor) {
    m_speed_factor = std::max(0.0, speed_factor);
    char message[50];
    snprintf(message, sizeof(message), "%g", m_speed_factor);
    g_speed_factor->setText(irr::core::stringw(message).c_str());
}

template <typename ScalarType>
void ChModalVisualSystemIrrlicht<ScalarType>::SetFixedFrequency(bool val) {
    m_fixed_frequency = val;
    g_fixed_frequency->setChecked(m_fixed_frequency);
}

template <typename ScalarType>
void ChModalVisualSystemIrrlicht<ScalarType>::AttachAssembly(const ChAssembly& assembly,
                                                             const ChMatrixDynamic<ScalarType>& eigvects,
                                                             const ChVectorDynamic<double>& freq) {
    m_assembly = const_cast<ChAssembly*>(&assembly);

    UpdateModes(eigvects, freq);
}

template <typename ScalarType>
void ChModalVisualSystemIrrlicht<ScalarType>::UpdateModes(const ChMatrixDynamic<ScalarType>& eigvects,
                                                          const ChVectorDynamic<double>& freq) {
    m_eigvects = &eigvects;
    m_freq = &freq;
    m_damping_ratios = nullptr;

    g_selected_mode->setMin(1);  // required to refit the scale
    g_selected_mode->setMax(m_eigvects->cols() - 1);
    SetMode(1);
}

template <typename ScalarType>
template <typename U>
typename std::enable_if<std::is_same<U, std::complex<double>>::value, void>::type
ChModalVisualSystemIrrlicht<ScalarType>::UpdateModes(const ChMatrixDynamic<ScalarType>& eigvects,
                                                     const ChVectorDynamic<double>& freq,
                                                     const ChVectorDynamic<double>& damping_ratios) {
    m_damping_ratios = &damping_ratios;
    UpdateModes(eigvects, freq);

    SetMode(1);  // required to print the damping ratio
}

// @} modal_vis

}  // end namespace modal
}  // end namespace chrono

#endif
