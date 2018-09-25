#include <QMouseEvent>
#include <QGuiApplication>

#include "NGLScene.h"
#include <ngl/NGLInit.h>
#include <ngl/Random.h>
#include <ngl/Util.h>
#include <cmath>
#include <ngl/ShaderLib.h>

#include <iostream>


NGLScene::NGLScene()
{
  // re-size the widget to that of the parent (in this case the GLFrame passed in on construction)
  setTitle("RVO2 Demo Blocks Space to Pause R to reset");
}


NGLScene::~NGLScene()
{
  std::cout<<"Shutting down NGL, removing VAO's and Shaders\n";
}



void NGLScene::resizeGL(int _w , int _h)
{
  m_win.width=_w*devicePixelRatio();
  m_win.height=_h*devicePixelRatio();
}

void NGLScene::setupSim()
{
  m_sim.reset(new RVO::RVOSimulator());

  /* Specify the global time step of the simulation. */
  m_sim->setTimeStep(0.25f);

  /* Specify the default parameters for agents that are subsequently added. */
  m_sim->setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f);

  /*
   * Add agents, specifying their start position, and store their goals on the
   * opposite side of the environment.
   */
  for (size_t i = 0; i < 6; ++i)
  {
    for (size_t j = 0; j < 6; ++j)
    {
      m_sim->addAgent(RVO::Vector2(55.0f + i * 10.0f,  55.0f + j * 10.0f));
      m_goals.push_back(RVO::Vector2(-75.0f, -75.0f));

      m_sim->addAgent(RVO::Vector2(-55.0f - i * 10.0f,  55.0f + j * 10.0f));
      m_goals.push_back(RVO::Vector2(75.0f, -75.0f));

      m_sim->addAgent(RVO::Vector2(55.0f + i * 10.0f, -55.0f - j * 10.0f));
      m_goals.push_back(RVO::Vector2(-75.0f, 75.0f));

      m_sim->addAgent(RVO::Vector2(-55.0f - i * 10.0f, -55.0f - j * 10.0f));
      m_goals.push_back(RVO::Vector2(75.0f, 75.0f));
    }
  }

  /*
   * Add (polygonal) obstacles, specifying their vertices in counterclockwise
   * order.
   */
  std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;

  obstacle1.push_back(RVO::Vector2(-10.0f, 40.0f));
  obstacle1.push_back(RVO::Vector2(-40.0f, 40.0f));
  obstacle1.push_back(RVO::Vector2(-40.0f, 10.0f));
  obstacle1.push_back(RVO::Vector2(-10.0f, 10.0f));

  obstacle2.push_back(RVO::Vector2(10.0f, 40.0f));
  obstacle2.push_back(RVO::Vector2(10.0f, 10.0f));
  obstacle2.push_back(RVO::Vector2(40.0f, 10.0f));
  obstacle2.push_back(RVO::Vector2(40.0f, 40.0f));

  obstacle3.push_back(RVO::Vector2(10.0f, -40.0f));
  obstacle3.push_back(RVO::Vector2(40.0f, -40.0f));
  obstacle3.push_back(RVO::Vector2(40.0f, -10.0f));
  obstacle3.push_back(RVO::Vector2(10.0f, -10.0f));

  obstacle4.push_back(RVO::Vector2(-10.0f, -40.0f));
  obstacle4.push_back(RVO::Vector2(-10.0f, -10.0f));
  obstacle4.push_back(RVO::Vector2(-40.0f, -10.0f));
  obstacle4.push_back(RVO::Vector2(-40.0f, -40.0f));

  m_sim->addObstacle(obstacle1);
  m_sim->addObstacle(obstacle2);
  m_sim->addObstacle(obstacle3);
  m_sim->addObstacle(obstacle4);

  /* Process the obstacles so that they are accounted for in the simulation. */
  m_sim->processObstacles();
}

void NGLScene::setPreferredVelocities()
{
  /*
   * Set the preferred velocity to be a vector of unit magnitude (speed) in the
   * direction of the goal.
   */
  for (size_t i = 0; i < m_sim->getNumAgents(); ++i)
  {
    RVO::Vector2 goalVector = m_goals[i] - m_sim->getAgentPosition(i);

    if (RVO::absSq(goalVector) > 1.0f)
    {
      goalVector = RVO::normalize(goalVector);
    }

    m_sim->setAgentPrefVelocity(i, goalVector);

    /*
     * Perturb a little to avoid deadlocks due to perfect symmetry.
     */
    ngl::Random *rng=ngl::Random::instance();
    rng->setSeed();
    auto angle = rng->randomNumber(2.0f * ngl::PI);
    auto dist =  rng->randomNumber(0.0001f );

    m_sim->setAgentPrefVelocity(i, m_sim->getAgentPrefVelocity(i) +
                              dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
  }
}

bool NGLScene::reachedGoal()
{
  /* Check if all agents have reached their goals. */
    for (size_t i = 0; i < m_sim->getNumAgents(); ++i)
    {
      if (RVO::absSq(m_sim->getAgentPosition(i) - m_goals[i]) > 20.0f * 20.0f)
      {
        return false;
      }
    }
    setupSim();
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
  ngl::NGLInit::instance();
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);			   // Grey Background
  // enable depth testing for drawing
  glEnable(GL_DEPTH_TEST);
  // enable multisampling for smoother drawing
  glEnable(GL_MULTISAMPLE);
  // now to load the shader and set the values
   // grab an instance of shader manager
   ngl::ShaderLib *shader=ngl::ShaderLib::instance();

  (*shader)["nglDiffuseShader"]->use();
  shader->setUniform("Colour",1.0f,1.0f,0.0f,1.0f);
  shader->setUniform("lightPos",1.0f,1.0f,1.0f);
  shader->setUniform("lightDiffuse",1.0f,1.0f,1.0f,1.0f);
  // Now we will create a basic Camera from the graphics library
  // This is a static camera so it only needs to be set once
  // First create Values for the camera position
  ngl::Vec3 from(0,1,15);
  ngl::Vec3 to(0,0,0);
  ngl::Vec3 up(0,1,0);
  // now load to our new camera
  m_view=ngl::lookAt(from,to,up);
  // set the shape using FOV 45 Aspect Ratio based on Width and Height
  // The final two are near and far clipping planes of 0.5 and 10
  m_project=ngl::perspective(50.0f,720.0f/576.0f,0.05f,350.0f);
  setupSim();
  ngl::VAOPrimitives::instance()->createTrianglePlane( "grid",200,200,10,10,ngl::Vec3::up());
  startTimer(1);

}


void NGLScene::loadMatricesToShader()
{
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();
  shader->use("nglDiffuseShader");
  ngl::Mat4 MV;
  ngl::Mat4 MVP;
  ngl::Mat3 normalMatrix;

  MV= m_view*
      m_globalTransformMatrix*
      m_bodyTransform;

  MVP= m_project*MV;
  normalMatrix=MV;
  normalMatrix.inverse().transpose();
  shader->setUniform("MVP",MVP);
  shader->setUniform("normalMatrix",normalMatrix);
}

void NGLScene::paintGL()
{
  // clear the screen and depth buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0,0,m_win.width,m_win.height);
  // grab an instance of the shader manager
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();
  (*shader)["nglDiffuseShader"]->use();

  // Rotation based on the mouse position for our global transform
  ngl::Mat4 rotX;
  ngl::Mat4 rotY;
  // create the rotation matrices
  rotX.rotateX(m_win.spinXFace);
  rotY.rotateY(m_win.spinYFace);
  // multiply the rotations
  m_globalTransformMatrix=rotY*rotX;
  // add the translations
  m_globalTransformMatrix.m_m[3][0] = m_modelPos.m_x;
  m_globalTransformMatrix.m_m[3][1] = m_modelPos.m_y;
  m_globalTransformMatrix.m_m[3][2] = m_modelPos.m_z;
  // now draw
  shader->use("nglDiffuseShader");
  for(auto g : m_goals)
  {
    shader->setUniform("Colour",0.0f,1.0f,0.0f,1.0f);

    ngl::Transformation t;
    t.setPosition(g.x(),0.0f,g.y());
    t.setScale(2.0,1.0,2.0);
    m_bodyTransform=t.getMatrix();
    loadMatricesToShader();
    ngl::VAOPrimitives::instance()->draw("cube");

  }

  for (size_t i = 0; i < m_sim->getNumAgents(); ++i)
  {
    shader->setUniform("Colour",1.0f,1.0f,0.0f,1.0f);
    ngl::Transformation t;
    RVO::Vector2 p=m_sim->getAgentPosition(i);
    RVO::Vector2 v=m_sim->getAgentVelocity(i);
    RVO::Vector2 next=p+v;
    RVO::Vector2 final=next-p;
    auto yrot=ngl::degrees(atan2(final.x(),final.y()));

    t.setRotation(0.0f,yrot,0.0f);

    t.setPosition(p.x(),0.0f,p.y());
    // match the radius of the agent
    t.setScale(1.5f, 1.5f,1.5f);
    m_bodyTransform=t.getMatrix();
    loadMatricesToShader();
    ngl::VAOPrimitives::instance()->draw("troll");
  }

  // hard coded draw of the Goals from the scene setup
  shader->setUniform("Colour",1.0f,0.0f,0.0f,1.0f);

  ngl::Transformation t;
  t.setPosition(-25.0,1.0,25);
  t.setScale(30.0f,3.0f,30.0f);
  m_bodyTransform=t.getMatrix();
  loadMatricesToShader();
  ngl::VAOPrimitives::instance()->draw("cube");

  t.setPosition(25.0,1.0,25);
  t.setScale(30.0f,3.0f,30.0f);
  m_bodyTransform=t.getMatrix();
  loadMatricesToShader();
  ngl::VAOPrimitives::instance()->draw("cube");

  t.setPosition(-25.0,1.0,-25);
  t.setScale(30.0f,3.0f,30.0f);
  m_bodyTransform=t.getMatrix();
  loadMatricesToShader();
  ngl::VAOPrimitives::instance()->draw("cube");

  t.setPosition(25.0,1.0,-25);
  t.setScale(30.0f,3.0f,30.0f);
  m_bodyTransform=t.getMatrix();
  loadMatricesToShader();
  ngl::VAOPrimitives::instance()->draw("cube");


  shader->use("nglColourShader");
  shader->setUniform("Colour",0.3f,0.3f,0.3f,1.0f);
  ngl::Mat4 MV;
  ngl::Mat4 MVP;
  m_bodyTransform.identity();
  m_bodyTransform.translate(0,-1,0);
  MV= m_view *
      m_globalTransformMatrix*
      m_bodyTransform;
  MVP= m_project*MV;

  shader->setUniform("MVP",MVP);
  ngl::VAOPrimitives::instance()->draw("grid");


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
