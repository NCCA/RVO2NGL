#include <QMouseEvent>
#include <QGuiApplication>

#include "NGLScene.h"
#include <ngl/NGLInit.h>
#include <ngl/ShaderLib.h>
#include <ngl/Random.h>
#include <iostream>


NGLScene::NGLScene()
{
  // re-size the widget to that of the parent (in this case the GLFrame passed in on construction)
  setTitle("RVO2 Demo Roadmap Space to Pause R to reset");
}


NGLScene::~NGLScene()
{
  std::cout<<"Shutting down NGL, removing VAO's and Shaders\n";
}

void NGLScene::resizeGL( int _w, int _h )
{
  m_cam.setShape( 45.0f, static_cast<float>( _w ) / _h, 0.05f, 350.0f );
  m_win.width  = static_cast<int>( _w * devicePixelRatio() );
  m_win.height = static_cast<int>( _h * devicePixelRatio() );
}

void NGLScene::setupSim()
{
  m_sim.reset(new RVO::RVOSimulator());

  /* Specify the global time step of the simulation. */
  m_sim->setTimeStep(0.25f);

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

  /* Add m_roadmap vertices. */
  RoadmapVertex v;

  /* Add the goal positions of agents. */
  v.position = RVO::Vector2(-75.0f, -75.0f);
  m_roadmap.push_back(v);
  v.position = RVO::Vector2(75.0f, -75.0f);
  m_roadmap.push_back(v);
  v.position = RVO::Vector2(-75.0f, 75.0f);
  m_roadmap.push_back(v);
  v.position = RVO::Vector2(75.0f, 75.0f);
  m_roadmap.push_back(v);

  /* Add m_roadmap vertices around the obstacles. */
  v.position = RVO::Vector2(-42.0f, -42.0f);
  m_roadmap.push_back(v);
  v.position = RVO::Vector2(-42.0f, -8.0f);
  m_roadmap.push_back(v);
  v.position = RVO::Vector2(-42.0f, 8.0f);
  m_roadmap.push_back(v);
  v.position = RVO::Vector2(-42.0f, 42.0f);
  m_roadmap.push_back(v);
  v.position = RVO::Vector2(-8.0f, -42.0f);
  m_roadmap.push_back(v);
  v.position = RVO::Vector2(-8.0f, -8.0f);
  m_roadmap.push_back(v);
  v.position = RVO::Vector2(-8.0f, 8.0f);
  m_roadmap.push_back(v);
  v.position = RVO::Vector2(-8.0f, 42.0f);
  m_roadmap.push_back(v);
  v.position = RVO::Vector2(8.0f, -42.0f);
  m_roadmap.push_back(v);
  v.position = RVO::Vector2(8.0f, -8.0f);
  m_roadmap.push_back(v);
  v.position = RVO::Vector2(8.0f, 8.0f);
  m_roadmap.push_back(v);
  v.position = RVO::Vector2(8.0f, 42.0f);
  m_roadmap.push_back(v);
  v.position = RVO::Vector2(42.0f, -42.0f);
  m_roadmap.push_back(v);
  v.position = RVO::Vector2(42.0f, -8.0f);
  m_roadmap.push_back(v);
  v.position = RVO::Vector2(42.0f, 8.0f);
  m_roadmap.push_back(v);
  v.position = RVO::Vector2(42.0f, 42.0f);
  m_roadmap.push_back(v);

  /* Specify the default parameters for agents that are subsequently added. */
  m_sim->setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f);

  /*
   * Add agents, specifying their start position, and store goals on the
   * opposite side of the environment (m_roadmap vertices).
   */
  for (size_t i = 0; i < 5; ++i)
  {
    for (size_t j = 0; j < 5; ++j)
    {
      m_sim->addAgent(RVO::Vector2(55.0f + i * 10.0f,  55.0f + j * 10.0f));
      m_goals.push_back(0);

      m_sim->addAgent(RVO::Vector2(-55.0f - i * 10.0f,  55.0f + j * 10.0f));
      m_goals.push_back(1);

      m_sim->addAgent(RVO::Vector2(55.0f + i * 10.0f, -55.0f - j * 10.0f));
      m_goals.push_back(2);

      m_sim->addAgent(RVO::Vector2(-55.0f - i * 10.0f, -55.0f - j * 10.0f));
      m_goals.push_back(3);
    }
  }

}


void NGLScene::buildRoadmap()
{
  /* Connect the m_roadmap vertices by edges if mutually visible. */
  for (int i = 0; i < static_cast<int>(m_roadmap.size()); ++i)
  {
    for (int j = 0; j < static_cast<int>(m_roadmap.size()); ++j)
    {
      if (m_sim->queryVisibility(m_roadmap[i].position, m_roadmap[j].position, m_sim->getAgentRadius(0)))
      {
        m_roadmap[i].neighbors.push_back(j);
      }
    }

    /*
     * Initialize the distance to each of the four goal vertices at infinity
     * (9e9f).
     */
    m_roadmap[i].distToGoal.resize(4, 9e9f);
  }

  /*
   * Compute the distance to each of the four goals (the first four vertices)
   * for all vertices using Dijkstra's algorithm.
   */
  for (int i = 0; i < 4; ++i)
  {
    std::multimap<float, int> Q;
    std::vector<std::multimap<float, int>::iterator> posInQ(m_roadmap.size(), Q.end());

    m_roadmap[i].distToGoal[i] = 0.0f;
    posInQ[i] = Q.insert(std::make_pair(0.0f, i));

    while (!Q.empty())
    {
      const int u = Q.begin()->second;
      Q.erase(Q.begin());
      posInQ[u] = Q.end();

      for (int j = 0; j < static_cast<int>(m_roadmap[u].neighbors.size()); ++j)
      {
        const int v = m_roadmap[u].neighbors[j];
        const float dist_uv = RVO::abs(m_roadmap[v].position - m_roadmap[u].position);

        if (m_roadmap[v].distToGoal[i] > m_roadmap[u].distToGoal[i] + dist_uv)
        {
          m_roadmap[v].distToGoal[i] = m_roadmap[u].distToGoal[i] + dist_uv;

          if (posInQ[v] == Q.end())
          {
            posInQ[v] = Q.insert(std::make_pair(m_roadmap[v].distToGoal[i], v));
          }
          else
          {
            Q.erase(posInQ[v]);
            posInQ[v] = Q.insert(std::make_pair(m_roadmap[v].distToGoal[i], v));
          }
        }
      }
    }
  }
}

void NGLScene::setPreferredVelocities()
{
  /*
     * Set the preferred velocity to be a vector of unit magnitude (speed) in the
     * direction of the visible roadmap vertex that is on the shortest path to the
     * goal.
     */
    for (int i = 0; i < static_cast<int>(m_sim->getNumAgents()); ++i)
    {
      float minDist = 9e9f;
      int minVertex = -1;

      for (int j = 0; j < static_cast<int>(m_roadmap.size()); ++j)
      {
        if (RVO::abs(m_roadmap[j].position - m_sim->getAgentPosition(i)) + m_roadmap[j].distToGoal[m_goals[i]] < minDist &&
          m_sim->queryVisibility(m_sim->getAgentPosition(i), m_roadmap[j].position, m_sim->getAgentRadius(i)))
        {

          minDist = RVO::abs(m_roadmap[j].position - m_sim->getAgentPosition(i)) + m_roadmap[j].distToGoal[m_goals[i]];
          minVertex = j;
        }
      }

      if (minVertex == -1)
      {
        /* No roadmap vertex is visible; should not happen. */
        m_sim->setAgentPrefVelocity(i, RVO::Vector2(0, 0));
      }
      else
      {
        if (RVO::absSq(m_roadmap[minVertex].position -
                       m_sim->getAgentPosition(i)) == 0.0f)
        {
          if (minVertex == m_goals[i])
          {
            m_sim->setAgentPrefVelocity(i, RVO::Vector2());
          }
          else
          {
            m_sim->setAgentPrefVelocity(i, RVO::normalize(m_roadmap[m_goals[i]].position - m_sim->getAgentPosition(i)));
          }
        }
        else
        {
          m_sim->setAgentPrefVelocity(i, RVO::normalize(m_roadmap[minVertex].position - m_sim->getAgentPosition(i)));
        }
      }

      /*
       * Perturb a little to avoid deadlocks due to perfect symmetry.
       */
      ngl::Random *rng=ngl::Random::instance();
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
      if (RVO::absSq(m_sim->getAgentPosition(i) - m_roadmap[m_goals[i]].position) > 20.0f * 20.0f)
      {
        return false;
      }
    }
    setupSim();
    buildRoadmap();
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
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();
  (*shader)["nglDiffuseShader"]->use();
  shader->setUniform("Colour",1.0f,1.0f,0.0f,1.0f);
  shader->setUniform("lightPos",1.0f,1.0f,1.0f);
  shader->setUniform("lightDiffuse",1.0f,1.0f,1.0f,1.0f);
  // Now we will create a basic Camera from the graphics library
  // This is a static camera so it only needs to be set once
  // First create Values for the camera position
  ngl::Vec3 from(0,1,10);
  ngl::Vec3 to(0,0,0);
  ngl::Vec3 up(0,1,0);
  // now load to our new camera
  m_cam.set(from,to,up);
  // set the shape using FOV 45 Aspect Ratio based on Width and Height
  // The final two are near and far clipping planes of 0.5 and 10
  m_cam.setShape(50.0f,720.0f/576.0f,0.05f,350.0f);
  setupSim();
  buildRoadmap();
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

  MV= m_cam.getViewMatrix() *
      m_globalTransformMatrix*
      m_bodyTransform;
  MVP= m_cam.getVPMatrix()*MV;
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

  for (size_t i = 0; i < m_sim->getNumAgents(); ++i)
  {
    shader->setUniform("Colour",1.0f,1.0f,0.0f,1.0f);
    ngl::Transformation t;
    RVO::Vector2 p=m_sim->getAgentPosition(i);
    RVO::Vector2 v=m_sim->getAgentVelocity(i);
    RVO::Vector2 next=p+v;
    RVO::Vector2 final=next-p;
    auto yrot=ngl::degrees(atan2f(final.x(),final.y()));

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
  MV= m_cam.getViewMatrix() *
      m_globalTransformMatrix *
      m_bodyTransform;
  MVP= m_cam.getVPMatrix()*MV;

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
  case Qt::Key_R : setupSim(); buildRoadmap(); break;
  case Qt::Key_Space : m_animate^=true; break;
  default : break;



  }
  // finally update the GLWindow and re-draw

    update();
}
