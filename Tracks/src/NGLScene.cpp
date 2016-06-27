#include <QMouseEvent>
#include <QGuiApplication>

#include "NGLScene.h"
#include <ngl/NGLInit.h>
#include <ngl/Random.h>
#include <ngl/Util.h>
#include <ngl/ShaderLib.h>
#include <ngl/NGLStream.h>
#include <ngl/Random.h>
#include <ngl/VAOFactory.h>
#include <ngl/MultiBufferVAO.h>

#include <iostream>
//----------------------------------------------------------------------------------------------------------------------
/// @brief the increment for x/y translation with mouse movement
//----------------------------------------------------------------------------------------------------------------------
constexpr float INCREMENT=0.5f;
//----------------------------------------------------------------------------------------------------------------------
/// @brief the increment for the wheel zoom
//----------------------------------------------------------------------------------------------------------------------
constexpr float ZOOM=2.0f;

NGLScene::NGLScene()
{
  // re-size the widget to that of the parent (in this case the GLFrame passed in on construction)
  setTitle("RVO2 Demo Blocks Space to Pause R to reset");
}


NGLScene::~NGLScene()
{
  std::cout<<"Shutting down NGL, removing VAO's and Shaders\n";
}

void NGLScene::resizeGL(QResizeEvent *_event)
{
  m_width=_event->size().width()*devicePixelRatio();
  m_height=_event->size().height()*devicePixelRatio();
}

void NGLScene::resizeGL(int _w , int _h)
{
  m_width=_w*devicePixelRatio();
  m_height=_h*devicePixelRatio();
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
    m_tracks.clear();
    m_perVertColour.clear();
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



void NGLScene::setColourArray(size_t _numColours)
{
  ngl::Random *rng=ngl::Random::instance();
  rng->setSeed();
  for(size_t i=0; i<_numColours; ++i)
  {
      m_colours.push_back(ngl::Vec4(rng->randomPositiveNumber(1.0f),
                          rng->randomPositiveNumber(1.0f),
                          rng->randomPositiveNumber(1.0f),
                          1.0f));
  }
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
  shader->setShaderParam4f("Colour",1,1,0,1);
  shader->setShaderParam3f("lightPos",1,1,1);
  shader->setShaderParam4f("lightDiffuse",1,1,1,1);
  // Now we will create a basic Camera from the graphics library
  // This is a static camera so it only needs to be set once
  // First create Values for the camera position
  ngl::Vec3 from(0,2,10);
  ngl::Vec3 to(0,0,0);
  ngl::Vec3 up(0,1,0);
  // now load to our new camera
  m_cam.set(from,to,up);
  // set the shape using FOV 45 Aspect Ratio based on Width and Height
  // The final two are near and far clipping planes of 0.5 and 10
  m_cam.setShape(45.0f,720.0f/576.0f,0.05f,350.0f);
  setupSim();
  ngl::VAOPrimitives::instance()->createTrianglePlane( "grid",250,250,10,10,ngl::Vec3::up());
  setColourArray(m_sim->getNumAgents());

  startTimer(1);


 // we are creating a shader called Phong to save typos
 // in the code create some constexpr
 constexpr auto shaderProgram="Track";
 constexpr auto vertexShader="TrackVertex";
 constexpr auto fragShader="TrackFragment";
 // create the shader program
 shader->createShaderProgram(shaderProgram);
 // now we are going to create empty shaders for Frag and Vert
 shader->attachShader(vertexShader,ngl::ShaderType::VERTEX);
 shader->attachShader(fragShader,ngl::ShaderType::FRAGMENT);
 // attach the source
 shader->loadShaderSource(vertexShader,"shaders/TrackVertex.glsl");
 shader->loadShaderSource(fragShader,"shaders/TrackFragment.glsl");
 // compile the shaders
 shader->compileShader(vertexShader);
 shader->compileShader(fragShader);
 // add them to the program
 shader->attachShaderToProgram(shaderProgram,vertexShader);
 shader->attachShaderToProgram(shaderProgram,fragShader);
 // now we have associated that data we can link the shader
 shader->linkProgramObject(shaderProgram);
 // and make it active ready to load values
 (*shader)[shaderProgram]->use();

}


void NGLScene::loadMatricesToShader()
{
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();
  shader->use("nglDiffuseShader");
  ngl::Mat4 MV;
  ngl::Mat4 MVP;
  ngl::Mat3 normalMatrix;

  MV=  m_bodyTransform*m_globalTransformMatrix*m_cam.getViewMatrix();
  MVP= MV*m_cam.getVPMatrix();
  normalMatrix=MV;
  normalMatrix.inverse();
  shader->setUniform("MVP",MVP);
  shader->setUniform("normalMatrix",normalMatrix);
}

void NGLScene::paintGL()
{
  // clear the screen and depth buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0,0,m_width,m_height);
  // grab an instance of the shader manager
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();

  // Rotation based on the mouse position for our global transform
  ngl::Mat4 rotX;
  ngl::Mat4 rotY;
  // create the rotation matrices
  rotX.rotateX(m_spinXFace);
  rotY.rotateY(m_spinYFace);
  std::cout<<rotX<<'\n'<<rotY<<'\n';
  std::cout<<m_spinXFace<<' '<<m_spinYFace<<'\n';
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
    shader->setUniform("Colour",m_colours[i]);
    ngl::Transformation t;
    RVO::Vector2 p=m_sim->getAgentPosition(i);
    RVO::Vector2 v=m_sim->getAgentVelocity(i);
    RVO::Vector2 next=p+v;
    RVO::Vector2 final=next-p;
    auto yrot=ngl::degrees(atan2(final.x(),final.y()));

    t.setRotation(0.0f,yrot,0.0f);

    t.setPosition(p.x(),0.0f,p.y());

    m_tracks.push_back(ngl::Vec3(p.x(),0.1f,p.y()));
    m_perVertColour.push_back(m_colours[i].toVec3());
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
  MV=  m_bodyTransform*m_globalTransformMatrix*m_cam.getViewMatrix();
  MVP= MV*m_cam.getVPMatrix();

  shader->setUniform("MVP",MVP);
  ngl::VAOPrimitives::instance()->draw("grid");

  // draw tracks

  ngl::AbstractVAO *vao= ngl::VAOFactory::createVAO("multiBufferVAO",GL_POINTS);
  vao->bind();

  // in this case we are going to set our data as the vertices above
  vao->setData(ngl::MultiBufferVAO::VertexData(m_tracks.size()*sizeof(ngl::Vec3),m_tracks[0].m_x));
  vao->setVertexAttributePointer(0,3,GL_FLOAT,0,0);

  vao->setData(ngl::MultiBufferVAO::VertexData(m_perVertColour.size()*sizeof(ngl::Vec3),m_perVertColour[0].m_x));
  vao->setVertexAttributePointer(1,3,GL_FLOAT,0,0);
  //glVertexAttrib4f(1,1,1,0,1);

  vao->setNumIndices(m_tracks.size());
  shader->use("Track");
  MV=  m_bodyTransform*m_globalTransformMatrix*m_cam.getViewMatrix();
  MVP= MV*m_cam.getVPMatrix();

  shader->setUniform("MVP",MVP);
  glPointSize(2.0);
  vao->draw();
  vao->unbind();
  vao->removeVAO();



}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mouseMoveEvent (QMouseEvent * _event)
{
  // note the method buttons() is the button state when event was called
  // this is different from button() which is used to check which button was
  // pressed when the mousePress/Release event is generated
  if(m_rotate && _event->buttons() == Qt::LeftButton)
  {
    int diffx=_event->x()-m_origX;
    int diffy=_event->y()-m_origY;
    m_spinXFace += (float) 0.5f * diffy;
    m_spinYFace += (float) 0.5f * diffx;
    m_origX = _event->x();
    m_origY = _event->y();
    update();

  }
        // right mouse translate code
  else if(m_translate && _event->buttons() == Qt::RightButton)
  {
    int diffX = (int)(_event->x() - m_origXPos);
    int diffY = (int)(_event->y() - m_origYPos);
    m_origXPos=_event->x();
    m_origYPos=_event->y();
    m_modelPos.m_x += INCREMENT * diffX;
    m_modelPos.m_y -= INCREMENT * diffY;
    update();

   }
}


//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mousePressEvent ( QMouseEvent * _event)
{
  // this method is called when the mouse button is pressed in this case we
  // store the value where the maouse was clicked (x,y) and set the Rotate flag to true
  if(_event->button() == Qt::LeftButton)
  {
    m_origX = _event->x();
    m_origY = _event->y();
    m_rotate =true;
  }
  // right mouse translate mode
  else if(_event->button() == Qt::RightButton)
  {
    m_origXPos = _event->x();
    m_origYPos = _event->y();
    m_translate=true;
  }

}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mouseReleaseEvent ( QMouseEvent * _event )
{
  // this event is called when the mouse button is released
  // we then set Rotate to false
  if (_event->button() == Qt::LeftButton)
  {
    m_rotate=false;
  }
        // right mouse translate mode
  if (_event->button() == Qt::RightButton)
  {
    m_translate=false;
  }
}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::wheelEvent(QWheelEvent *_event)
{

  // check the diff of the wheel position (0 means no change)
  if(_event->delta() > 0)
  {
    m_modelPos.m_z+=ZOOM;
  }
  else if(_event->delta() <0 )
  {
    m_modelPos.m_z-=ZOOM;
  }
  update();
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
  case Qt::Key_R : setupSim(); m_tracks.clear(); m_perVertColour.clear();break;
  case Qt::Key_Space : m_animate^=true; break;
  default : break;



  }
  // finally update the GLWindow and re-draw

    update();
}
