#ifndef NGLSCENE_H_
#define NGLSCENE_H_
#include <ngl/Transformation.h>
#include "WindowParams.h"
#include <QOpenGLWindow>
#include <memory>
#include "RVO.h"
//----------------------------------------------------------------------------------------------------------------------
/// @file NGLScene.h
/// @brief this class inherits from the Qt OpenGLWindow and allows us to use NGL to draw OpenGL
/// @author Jonathan Macey
/// @version 1.0
/// @date 10/9/13
/// Revision History :
/// This is an initial version used for the new NGL6 / Qt 5 demos
/// @class NGLScene
/// @brief our main glwindow widget for NGL applications all drawing elements are
/// put in this file
//----------------------------------------------------------------------------------------------------------------------

class NGLScene : public QOpenGLWindow
{
  public:
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief ctor for our NGL drawing class
    /// @param [in] parent the parent window to the class
    //----------------------------------------------------------------------------------------------------------------------
    NGLScene();
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief dtor must close down ngl and release OpenGL resources
    //----------------------------------------------------------------------------------------------------------------------
    ~NGLScene();
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief the initialize class is called once when the window is created and we have a valid GL context
    /// use this to setup any default GL stuff
    //----------------------------------------------------------------------------------------------------------------------
    void initializeGL();
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this is called everytime we want to draw the scene
    //----------------------------------------------------------------------------------------------------------------------
    void paintGL();

private:
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this is called everytime we resize the window
    //----------------------------------------------------------------------------------------------------------------------
    void resizeGL(int _w, int _h);

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief Qt Event called when a key is pressed
    /// @param [in] _event the Qt event to query for size etc
    //----------------------------------------------------------------------------------------------------------------------
    void keyPressEvent(QKeyEvent *_event);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this method is called every time a mouse is moved
    /// @param _event the Qt Event structure
    //----------------------------------------------------------------------------------------------------------------------
    void mouseMoveEvent (QMouseEvent * _event );
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this method is called everytime the mouse button is pressed
    /// inherited from QObject and overridden here.
    /// @param _event the Qt Event structure
    //----------------------------------------------------------------------------------------------------------------------
    void mousePressEvent ( QMouseEvent *_event);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this method is called everytime the mouse button is released
    /// inherited from QObject and overridden here.
    /// @param _event the Qt Event structure
    //----------------------------------------------------------------------------------------------------------------------
    void mouseReleaseEvent ( QMouseEvent *_event );

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this method is called everytime the mouse wheel is moved
    /// inherited from QObject and overridden here.
    /// @param _event the Qt Event structure
    //----------------------------------------------------------------------------------------------------------------------
    void wheelEvent( QWheelEvent *_event);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief the windows params such as mouse and rotations etc
    //----------------------------------------------------------------------------------------------------------------------
    WinParams m_win;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief Our Camera
    //----------------------------------------------------------------------------------------------------------------------
    ngl::Mat4 m_view;
    ngl::Mat4 m_project;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief body transform matrix
    //----------------------------------------------------------------------------------------------------------------------
    ngl::Mat4 m_bodyTransform;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief world / mouse transform
    //----------------------------------------------------------------------------------------------------------------------
    ngl::Mat4 m_globalTransformMatrix;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief the model position for mouse movement
    //----------------------------------------------------------------------------------------------------------------------
    ngl::Vec3 m_modelPos;

    /// the sim
    std::unique_ptr<RVO::RVOSimulator > m_sim;
    std::vector <RVO::Vector2> m_goals;
    void setupSim();
    void setPreferredVelocities();
    bool reachedGoal() ;
    void timerEvent(QTimerEvent *);
    void loadMatricesToShader();
    bool m_animate=true;

    std::vector <ngl::Vec3> m_tracks;
    // array of unique colours for agents
    std::vector <ngl::Vec4> m_colours;
    std::vector <ngl::Vec3> m_perVertColour;
    void setColourArray(size_t _numColours);

};



#endif
