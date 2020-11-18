#include <QMouseEvent>
#include <QGuiApplication>

#include "NGLScene.h"
#include <ngl/NGLInit.h>
#include <ngl/ShaderLib.h>
#include <iostream>
#include <cmath>

NGLScene::NGLScene()
{
  // re-size the widget to that of the parent (in this case the GLFrame passed in on construction)
  setTitle("RVO2 Demo Circle Space to Pause R to reset");
}


NGLScene::~NGLScene()
{
  std::cout<<"Shutting down NGL, removing VAO's and Shaders\n";
}

void NGLScene::resizeGL( int _w, int _h )
{
  m_project=ngl::perspective( 45.0f, static_cast<float>( _w ) / _h, 0.05f, 350.0f );
  m_win.width  = static_cast<int>( _w * devicePixelRatio() );
  m_win.height = static_cast<int>( _h * devicePixelRatio() );
}

void NGLScene::setupSim()
{
  m_sim.reset(new RVO::RVOSimulator());
  /* Specify the global time step of the simulation. */
  m_sim->setTimeStep(0.25f);
  /* Specify the default parameters for agents that are subsequently added. */
  m_sim->setAgentDefaults(15.0f, 10, 10.0f, 10.0f, 1.5f, 2.0f);

  /*
   * Add agents, specifying their start position, and store their goals on the
   * opposite side of the environment.
   */
  for (size_t i = 0; i < 250; ++i)
  {
    m_sim->addAgent(200.0f *
                  RVO::Vector2(cosf(i * 2.0f * M_PI / 250.0f),
                               sinf(i * 2.0f * M_PI / 250.0f)));
    m_goals.push_back(-m_sim->getAgentPosition(i));
  }
}

void NGLScene::setPreferredVelocities()
{
  /*
   * Set the preferred velocity to be a vector of unit magnitude (speed) in the
   * direction of the goal.
   */
  for (int i = 0; i < static_cast<int>(m_sim->getNumAgents()); ++i)
  {
    RVO::Vector2 goalVector = m_goals[i] - m_sim->getAgentPosition(i);

    if (RVO::absSq(goalVector) > 1.0f)
    {
      goalVector = RVO::normalize(goalVector);
    }

    m_sim->setAgentPrefVelocity(i, goalVector);
  }
}

bool NGLScene::reachedGoal() const
{
  for (size_t i = 0; i < m_sim->getNumAgents(); ++i)
  {
    if (RVO::absSq(m_sim->getAgentPosition(i) - m_goals[i]) > m_sim->getAgentRadius(i) * m_sim->getAgentRadius(i))
    {
      return false;
    }
  }

    return true;
}

void NGLScene::timerEvent(QTimerEvent *)
{
  if(!reachedGoal() && m_animate)
  {
    setPreferredVelocities();
    m_sim->doStep();
  }
  update();
}


void NGLScene::initializeGL()
{
  // we need to initialise the NGL lib which will load all of the OpenGL functions, this must
  // be done once we have a valid GL context but before we call any GL commands. If we dont do
  // this everything will crash
  ngl::NGLInit::initialize();
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);			   // Grey Background
  // enable depth testing for drawing
  glEnable(GL_DEPTH_TEST);
  // enable multisampling for smoother drawing
  glEnable(GL_MULTISAMPLE);
  ngl::ShaderLib::use("nglDiffuseShader");
  ngl::ShaderLib::setUniform("Colour",1.0f,1.0f,0.0f,1.0f);
  ngl::ShaderLib::setUniform("lightPos",1.0f,1.0f,1.0f);
  ngl::ShaderLib::setUniform("lightDiffuse",1.0f,1.0f,1.0f,1.0f);
  // Now we will create a basic Camera from the graphics library
  // This is a static camera so it only needs to be set once
  // First create Values for the camera position
  ngl::Vec3 from(0,1,10);
  ngl::Vec3 to(0,0,0);
  ngl::Vec3 up(0,1,0);
  // now load to our new camera
  m_view=ngl::lookAt(from,to,up);
  // set the shape using FOV 45 Aspect Ratio based on Width and Height
  // The final two are near and far clipping planes of 0.5 and 10
  m_project=ngl::perspective(50.0f,720.0f/576.0f,0.05f,350.0f);
  setupSim();
  ngl::VAOPrimitives::createTrianglePlane( "grid",600,600,100,100,ngl::Vec3::up());
  startTimer(1);

}


void NGLScene::loadMatricesToShader()
{
  ngl::ShaderLib::use("nglDiffuseShader");

  ngl::Mat4 MV;
  ngl::Mat4 MVP;
  ngl::Mat3 normalMatrix;

  MV= m_view*
      m_globalTransformMatrix*
      m_bodyTransform;


  MVP= m_project*MV;
  normalMatrix=MV;
  normalMatrix.inverse().transpose();
  ngl::ShaderLib::setUniform("MVP",MVP);
  ngl::ShaderLib::setUniform("normalMatrix",normalMatrix);
}

void NGLScene::paintGL()
{
  // clear the screen and depth buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0,0,m_win.width,m_win.height);
  ngl::ShaderLib::use("nglDiffuseShader");

  // Rotation based on the mouse position for our global transform
  ngl::Mat4 rotX;
  ngl::Mat4 rotY;
  // create the rotation matrices
  rotX.rotateX(m_win.spinXFace);
  rotY.rotateY(m_win.spinYFace);
  // multiply the rotations
  m_globalTransformMatrix=rotX*rotY;
  // add the translations
  m_globalTransformMatrix.m_m[3][0] = m_modelPos.m_x;
  m_globalTransformMatrix.m_m[3][1] = m_modelPos.m_y;
  m_globalTransformMatrix.m_m[3][2] = m_modelPos.m_z;
  // now draw
  for(auto g : m_goals)
  {
    ngl::Transformation t;
    t.setPosition(g.x(),0.0f,g.y());
    m_bodyTransform=t.getMatrix();
    loadMatricesToShader();
    ngl::VAOPrimitives::draw("cube");

  }


  for (size_t i = 0; i < m_sim->getNumAgents(); ++i)
  {
    ngl::Transformation t;
    RVO::Vector2 p=m_sim->getAgentPosition(i);
    RVO::Vector2 v=m_sim->getAgentVelocity(i);
    RVO::Vector2 next=p+v;
    RVO::Vector2 final=next-p;
    auto yrot=ngl::degrees(atan2f(final.x(),final.y()));

    t.setPosition(p.x(),0.0f,p.y());
    t.setRotation(0.0f,yrot,0.0f);
    // match the radius of the agent
    t.setScale(1.5f, 1.5f,1.5f);
    m_bodyTransform=t.getMatrix();
    loadMatricesToShader();
    ngl::VAOPrimitives::draw("troll");
  }

  ngl::ShaderLib::use("nglColourShader");
  ngl::ShaderLib::setUniform("Colour",0.3f,0.3f,0.3f,1.0f);
  ngl::Mat4 MV;
  ngl::Mat4 MVP;
  m_bodyTransform.identity();
  m_bodyTransform.translate(0,-1,0);
  MV= m_view *
      m_globalTransformMatrix*
      m_bodyTransform;

  MVP=m_project*MV;

  ngl::ShaderLib::setUniform("MVP",MVP);
  ngl::VAOPrimitives::draw("grid");


}

//----------------------------------------------------------------------------------------------------------------------

void NGLScene::keyPressEvent(QKeyEvent *_event)
{
  // this method is called every time the main window recives a key event.
  // we then switch on the key value and set the camera in the GLWindow
  switch (_event->key())
  {
  // escape key to quite
  case Qt::Key_Escape : QGuiApplication::exit(EXIT_SUCCESS); break;
  case Qt::Key_R : setupSim(); break;
  case Qt::Key_Space : m_animate^=true; break;
  default : break;



  }
  // finally update the GLWindow and re-draw

    update();
}
