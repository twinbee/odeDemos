#include <iostream>
#include <cmath>
#include <ode/ode.h>

#include <GL/gl.h>
#include <GL/glut.h>

using namespace std;
 
 double PI = 3.1415927;

 const int n = 500; //number of bodies
 int rowlen = int(sqrt(n));

const bool wallson = true;

    dBodyID bods[n];
    dGeomID	 bodsG[n]; 	//we'll need a gemoetry describing each body

    dReal xs = 0.0+rowlen/2.0;
    dReal ys = 0.0-rowlen/2.0;

    dWorldID W;

dSpaceID S;		// we'll use only one space for collision detection

dBodyID  walls[4];		//we'll also need to detect a collision with the floor/ceiling
dGeomID  wallsG[4];		//we'll also need to detect a collision with the floor/ceiling

dJointGroupID contacts;
dContactGeom contactAry[255]; //well need to store a list of contacts temporarily
//we'll need to keep a group listing of all contacting points for collision handling

const float wal_len = 100.0;

inline double d2r(double a) {return 0.0174532925 * a;}
inline double r2d(double a) {return 0.0174532925 / a;}

void ODEtoOGL(const dReal* p, const dReal* R, float Element[16])
{
  Element[0]  = R[0]; Element[1]  = R[4]; Element[2]  = R[8]; Element[3]  = 0;
  Element[4]  = R[1]; Element[5]  = R[5]; Element[6]  = R[9]; Element[7]  = 0;
  Element[8]  = R[2]; Element[9]  = R[6]; Element[10] = R[10];Element[11] = 0;
  Element[12] = p[0]; Element[13] = p[1]; Element[14] = p[2]; Element[15] = 1;
}

void nearCallback (void * data, dGeomID o1, dGeomID o2)
{
 int i;
 const int MAX_CONTACTS = 1;
 dBodyID b1 = dGeomGetBody(o1);
 dBodyID b2 = dGeomGetBody(o2);
 if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;
                                                                                                     
 dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
 for (i=0; i<MAX_CONTACTS; i++)
 {
  contact[i].surface.mode = dContactBounce;
  contact[i].surface.mu = 1.0;
  contact[i].surface.bounce = 0.01;
  contact[i].surface.bounce_vel = 0.8;
 }

 if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom, sizeof(dContact)))
 {
  dMatrix3 RI;
  dRSetIdentity (RI);
  const dReal ss[3] = {0.02,0.02,0.02};

  for (i=0; i<numc; i++)
  {
   dJointID c = dJointCreateContact (W,contacts, contact+i);
   dJointAttach (c,b1,b2);
  }
 }
}

void InitGL ( GLvoid )     // Create Some Everyday Functions
{
	glShadeModel(GL_SMOOTH);							// Enable Smooth Shading
	glClearColor(0.0f, 0.0f, 0.0f, 0.5f);				// Black Background
	glClearDepth(1.0f);									// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
	glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
	glEnable ( GL_COLOR_MATERIAL );
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

/*
    W = dWorldCreate();
    
    //dWorldSetGravity (W, 0.0, 0.0, -9.82);

    S = dSimpleSpaceCreate(S); // Create a space object
*/

if (wallson) {

    wallsG[0] = dCreateBox (S, wal_len, 1.0, 10.0);
    wallsG[1] = dCreateBox (S, wal_len, 1.0, 10.0);
    wallsG[2] = dCreateBox (S, wal_len, 1.0, 10.0);
    wallsG[3] = dCreateBox (S, wal_len, 1.0, 10.0);

    dGeomSetPosition (wallsG[0], -wal_len/2.0+2, 20, 0.0);
    dGeomSetPosition (wallsG[1], wal_len/2.0-2, 20, 0.0);
    dGeomSetPosition (wallsG[2], -wal_len/2.0+2, -20, 0.0);
    dGeomSetPosition (wallsG[3], wal_len/2.0-2, -20, 0.0);

    dMatrix3 R;
    dRFromAxisAndAngle (R, 0.0, 0.0, 1.0, d2r(-10.0));
    dGeomSetRotation (wallsG[0], R);
    dGeomSetRotation (wallsG[3], R);
    dRFromAxisAndAngle (R, 0.0, 0.0, 1.0, d2r(10.0));
    dGeomSetRotation (wallsG[1], R);
    dGeomSetRotation (wallsG[2], R);
}
/*
    contacts = dJointGroupCreate(0); //create an empty group of contacts for sotring contact joints

    for (int i = 0; i < n; i++)
    {
     bods[i] = dBodyCreate (W);
     if (i%(rowlen)) {xs++;} else { ys++; xs -= rowlen-1; } 
     dBodySetPosition (bods[i], 2.0*xs, 2.0*ys, 0.0);
     bodsG[i] = dCreateSphere(S, 1.0);    
     dGeomSetBody(bodsG[i], bods[i]);
    }
 */   
}

void display ( void )   // Create The Display Function
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	// Clear Screen And Depth Buffer
  glLoadIdentity();									// Reset The Current Modelview Matrix

  glColor3f(1.0, 0.0, 1.0);

  glTranslatef(0.0f,0.0f,-100.0f);

  for (int i = 0; i < n; i++)
  {
   const dReal * pos = dBodyGetPosition (bods[i]);
   const dReal pos_[3] = {pos[0] - (!(i%2) ? 50.0 : -50.0), pos[1], pos[2]};
   float len = sqrt(pos_[0]*pos_[0] + pos_[1]*pos_[1] + pos_[2]*pos_[2] );
   dBodyAddForce (bods[i], -pos_[0]/(len*2.0), -pos_[1]/(len*2.0), -pos_[2]/(len*2.0));
  }

  dSpaceCollide(S, contacts, nearCallback);
  dWorldQuickStep (W, 0.2);
  
  dJointGroupEmpty(contacts); //empty the list of joint contacts

  for (int i = 0; i < n; i++)
  {
   const dReal * pos = dBodyGetPosition (bods[i]);
   glPushMatrix();
   glTranslatef (pos[0], pos[1], pos[2]);
   glutSolidSphere(1.0f, 10, 10);
   glPopMatrix();
  }

if (wallson){
  for (int i = 0; i < 4; i++)
  {
   const dReal * pos = dGeomGetPosition (wallsG[i]);
   const dReal * rot = dGeomGetRotation (wallsG[i]);  //might not work
   glPushMatrix();
   //glTranslatef (pos[0], pos[1], pos[2]);
   float out[16];
   ODEtoOGL(pos, rot, out);
   glMultMatrixf(out);
   glBegin(GL_QUADS);
    glVertex3f(-wal_len/2.0, 0.5, 0.0);
    glVertex3f( wal_len/2.0, 0.5, 0.0);
    glVertex3f( wal_len/2.0, -0.5, 0.0);
    glVertex3f(-wal_len/2.0, -0.5, 0.0);
   glEnd();
   glPopMatrix();
  }
}
  glutSwapBuffers ( );

}

void reshape ( int width , int height )   // Create The Reshape Function (the viewport)
{
  if (height==0)										// Prevent A Divide By Zero By
	{
		height=1;										// Making Height Equal One
	}

	glViewport(0,0,width,height);						// Reset The Current Viewport

	glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
	glLoadIdentity();									// Reset The Projection Matrix

	// Calculate The Aspect Ratio Of The Window
	gluPerspective(45.0f,(GLfloat)width/(GLfloat)height,0.1f,1000.0f);

	glMatrixMode(GL_MODELVIEW);							// Select The Modelview Matrix
	glLoadIdentity();									
}

void keyboard ( unsigned char key, int x, int y )  
{
  switch ( key ) {        
    case 27:        // When Escape Is Pressed...
      exit ( 0 );   // Exit The Program
      break;        // Ready For Next Case
    default:        // Now Wrap It Up
      break;
  }
}

void arrow_keys ( int a_keys, int x, int y )
 // Create Special Function (required for arrow keys)
{
  switch ( a_keys ) {
    case GLUT_KEY_UP:     // When Up Arrow Is Pressed...
      break;
    case GLUT_KEY_DOWN:               // When Down Arrow Is Pressed...
      break;
    default:
      break;
  }
}


int main(int argc, char *argv[])
{ 

 glutInit ( &argc, argv );

 glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE ); // Display Mode
 glutInitWindowSize  ( 800, 600 );
 glutCreateWindow    ( "+--++--+" );

 InitGL ();

 glutDisplayFunc     ( display );  // Matching Earlier Functions To Their Counterparts
 glutReshapeFunc     ( reshape );
 glutKeyboardFunc    ( keyboard );

 glutIdleFunc     ( display );
 glutMainLoop        ( );          // Initialize The Main Loop                  
 
    for (int i = 0; i < n; i++)
    {
         dBodyDestroy (bods[i]);
    }

    dWorldDestroy(W);
    dCloseODE();

    return EXIT_SUCCESS;
}
