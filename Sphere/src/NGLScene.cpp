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
  setTitle("RVO2 Demo Sphere Space to Pause R to reset");
}


NGLScene::~NGLScene()
{
  std::cout<<"Shutting down NGL, removing VAO's and Shaders\n";
}

void NGLScene::resizeGL( int _w, int _h )
{
  m_view=ngl::perspective( 45.0f, static_cast<float>( _w ) / _h, 0.05f, 350.0f );
  m_win.width  = static_cast<int>( _w * devicePixelRatio() );
  m_win.height = static_cast<int>( _h * devicePixelRatio() );
}

void NGLScene::setupSim()
{
  m_sim.reset(new RVO::RVOSimulator());
  /* Specify the global time step of the simulation. */
  m_sim->setTimeStep(0.5f);
  /* Specify the default parameters for agents that are subsequently added. */
  m_sim->setAgentDefaults(15.0f, 10, 10.0f, 1.5f, 2.0f);

  /* Add agents, specifying their start position, and store their goals on the opposite side of the environment. */
    for (float a = 0; a < M_PI; a += 0.1f)
    {
      const float z = 100.0f * std::cos(a);
      const float r = 100.0f * std::sin(a);

      for (size_t i = 0; i < r / 2.5f; ++i)
      {
        const float x = r * cosf(i * 2.0f * M_PI / (r / 2.5f));
        const float y = r * sinf(i * 2.0f * M_PI / (r / 2.5f));

        m_sim->addAgent(RVO::Vector3(x, y, z));
        m_goals.push_back(-m_sim->getAgentPosition(m_sim->getNumAgents() - 1));
      }
    }
}

void NGLScene::setPreferredVelocities()
{
  /* Set the preferred velocity to be a vector of unit magnitude (speed) in the direction of the goal. */
    for (size_t i = 0; i < m_sim->getNumAgents(); ++i)
    {
      RVO::Vector3 goalVector = m_goals[i] - m_sim->getAgentPosition(i);

      if (RVO::absSq(goalVector) > 1.0f) {
        goalVector = RVO::normalize(goalVector);
      }

      m_sim->setAgentPrefVelocity(i, goalVector);
    }
}

bool NGLScene::reachedGoal()
{
  /* Check if all agents have reached their goals. */
    for (size_t i = 0; i < m_sim->getNumAgents(); ++i)
    {
      if (RVO::absSq(m_sim->getAgentPosition(i) - m_goals[i]) > 4.0f * m_sim->getAgentRadius(i) * m_sim->getAgentRadius(i))
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
  ngl::NGLInit::initialize();
  glViewport(0,0,m_win.width,m_win.height);
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);			   // Grey Background
  // enable depth testing for drawing
  glEnable(GL_DEPTH_TEST);
  // enable multisampling for smoother drawing
  glEnable(GL_MULTISAMPLE);
  ngl::ShaderLib::use("nglDiffuseShader" );
  ngl::ShaderLib::setUniform("Colour",1.0f,1.0f,0.0f,1.0f);
  ngl::ShaderLib::setUniform("lightPos",1.0f,1.0f,1.0f);
  ngl::ShaderLib::setUniform("lightDiffuse",1.0f,1.0f,1.0f,1.0f);
  // Now we will create a basic Camera from the graphics library
  // This is a static camera so it only needs to be set once
  // First create Values for the camera position
  ngl::Vec3 from(0,0,260);
  ngl::Vec3 to(0,0,0);
  ngl::Vec3 up(0,1,0);
  // now load to our new camera
  m_view=ngl::lookAt(from,to,up);
  // set the shape using FOV 45 Aspect Ratio based on Width and Height
  // The final two are near and far clipping planes of 0.5 and 10
  m_project=ngl::perspective(50.0f,720.0f/576.0f,0.05f,350.0f);
  ngl::VAOPrimitives::createSphere("sphere",1.5,40);
  setupSim();
  buildVAO();
  startTimer(0);


}


void NGLScene::loadMatricesToShader()
{
  ngl::Mat4 MV;
  ngl::Mat4 MVP;
  ngl::Mat3 normalMatrix;

  MV= m_view *
      m_globalTransformMatrix *
      m_bodyTransform ;
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
  m_vao->bind();
  for (size_t i = 0; i < m_sim->getNumAgents(); ++i)
  {
    ngl::Transformation t;
    RVO::Vector3 p=m_sim->getAgentPosition(i);
    RVO::Vector3 v=m_sim->getAgentVelocity(i);
    RVO::Vector3 next=p+v;

    RVO::Vector3 f=next-p;
    ngl::Real x=f.x();
    ngl::Real y=f.y();
    ngl::Real z=f.z();

    //using spherical geometry we calculate the rotation based on the New Point
    auto yrot=ngl::degrees(atan2f(x,z))+180.0f; // Now convert from radians to deg

    // Now for the zrot
    auto r=sqrtf(x*x+y*y+z*z);
    auto xrot=ngl::degrees(asinf(y/r));

    t.setPosition(p.x(),p.y(),p.z());
    t.setScale(1.5f,1.5f,1.5f);
    t.setRotation(xrot,yrot,0.0f);

    m_bodyTransform=t.getMatrix();
    loadMatricesToShader();
    m_vao->draw();
    //ngl::VAOPrimitives::draw("sphere");
    std::cout<<"draw\n";
  }
  m_vao->unbind();

}




void NGLScene::buildVAO()
{
  std::vector<ngl::Vec3> verts=
  {
    ngl::Vec3(0,1,1),
    ngl::Vec3(0,0,-1),
    ngl::Vec3(-0.5,0,1),
    ngl::Vec3(0,1,1),
    ngl::Vec3(0,0,-1),
    ngl::Vec3(0.5,0,1),
    ngl::Vec3(0,1,1),
    ngl::Vec3(0,0,1.5),
    ngl::Vec3(-0.5,0,1),
    ngl::Vec3(0,1,1),
    ngl::Vec3(0,0,1.5),
    ngl::Vec3(0.5,0,1)

  };
  std::cout<<"Initial "<<verts.size()<<'\n';
  ngl::Vec3 n=ngl::calcNormal(verts[2],verts[1],verts[0]);
  verts.push_back(n);
  verts.push_back(n);
  verts.push_back(n);
  n=ngl::calcNormal(verts[3],verts[4],verts[5]);
  verts.push_back(n);
  verts.push_back(n);
  verts.push_back(n);

  n=ngl::calcNormal(verts[6],verts[7],verts[8]);
  verts.push_back(n);
  verts.push_back(n);
  verts.push_back(n);

  n=ngl::calcNormal(verts[11],verts[10],verts[9]);
  verts.push_back(n);
  verts.push_back(n);
  verts.push_back(n);


  std::cout<<"sizeof(verts) "<<sizeof(verts)<<" sizeof(ngl::Vec3) "<<sizeof(ngl::Vec3)<<"\n";
  // create a vao as a series of GL_TRIANGLES
  m_vao=ngl::VAOFactory::createVAO(ngl::simpleVAO,GL_TRIANGLES);
  m_vao->bind();

  // in this case we are going to set our data as the vertices above
  m_vao->setData(ngl::SimpleVAO::VertexData(verts.size()*sizeof(ngl::Vec3),verts[0].m_x));
  // now we set the attribute pointer to be 0 (as this matches vertIn in our shader)

  m_vao->setVertexAttributePointer(0,3,GL_FLOAT,0,0);

  // now we set the attribute pointer to be 1 (as this matches normal in our shader)
  // as we cast to ngl::Real for offset use 12 * 3 (as in x,y,z is 3 floats)
  m_vao->setVertexAttributePointer(1,3,GL_FLOAT,0,12*3);
  // divide by 2 as we have both verts and normals
  m_vao->setNumIndices(verts.size()/2);

 // now unbind
  m_vao->unbind();


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
