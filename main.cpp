

// Sample utils
#include "main.h"			// window system 
#include "nv_gui.h"			// gui system
#include <GL/glew.h>
#include <algorithm>

#include "joints.h"

#define MOVE_GOAL_XY	1
#define MOVE_GOAL_XZ	2

class Sample : public NVPWindow {
public:
	virtual bool init();
	virtual void display();
	virtual void reshape(int w, int h);
	virtual void motion(int x, int y, int dx, int dy);
	virtual void keyboardchar(unsigned char key, int mods, int x, int y);
	virtual void mouse (NVPWindow::MouseButton button, NVPWindow::ButtonAction state, int mods, int x, int y);

	void		Reset ();

	int			mouse_down;
	int			mouse_action;
	int			m_adjust;
	bool		m_bRunInv;

	Camera3D*	cam;
	Vector3DF	m_goal;

	Joints		m_joints;	
};


/*void Sample::start_guis (int w, int h)
{
	setview2D ( w, h );
	guiSetCallback ( 0x0 );		
	addGui ( 10, h-30, 130, 20, "Overlay", GUI_CHECK, GUI_BOOL, &m_overlay, 0.f, 1.f );		
}*/

void Sample::Reset ()
{
	// J1) Shoulder
	//    Ball pivot: X-axis up, Y-axis outward, Z-axis forward 
	//                             T-pose       Minimum   Maximum
	//    X-axis, swing fwd/back:  0=out right, -60=back, +130=forward
	//    Y-axis, shoulder twist:  0=out front, -80=down, +90=up     ** +45 typical **
	//    Z-axis, swing up/down:  90=out right,  -5=over, 200=down, 0=straight up, 90=out,
	// J2) Elbow:
	//    Hinge jnt:  X-axis up, Y-axis outward, Z-axis forward 
	//    X-axis, rotate:		   0=out,        +3=limit, +150=closed
	// J3) Wrist: 
	//    Ball pivot: X-axis up, Y-axis outward, Z-axis forward  (X = thumb up)
	//    X-axis, palm up/down:    0=out flat,  -80=palm extend, +80=palm down
	//    Y-axis, wrist twist:     0=thumb up,  -90=thumb fwd,   +60=thumb back
	//    Z-axis, wrist yaw:       0=out strt,  -25=cantor inwd, +25=cantor outwd
	
	// create joints
	m_joints.Clear ();
	
	//---- Human Arm
	m_joints.AddJoint ( "back", 5, Vector3DF(0,0,0), 0, 0, 0 );
	m_joints.AddJoint ( "shoulder", 3, Vector3DF(0,0,90), 1, 1, 1 );	
	m_joints.AddJoint ( "elbow", 3, Vector3DF(0,0,0), 1, 0, 0 );
	//m_joints.AddJoint ( "J3", 2, Vector3DF(0,0,0), 1, 1, 1 );	 
	m_joints.SetLimits ( 1, Vector3DF(-60, -80, 5), Vector3DF(130, 45, 200) );	 	// shoulder
	m_joints.SetLimits ( 2, Vector3DF( +5, 0, 0),    Vector3DF(150, 0, 0) );		// elbow
	//m_joints.SetLimits ( 3, Vector3DF(-80, -90, -25),Vector3DF(+80,+60,+25) );		// wrist
	

	//---- Robot Arm
	/*m_joints.AddJoint ( "J0", 1, Vector3DF(0,0,0), 0, 1, 0 );
	m_joints.AddJoint ( "J1", 4, Vector3DF(0,0,0), 1, 0, 0 );	
	m_joints.AddJoint ( "J2", 4, Vector3DF(0,0,0), 1, 0, 0 );
	m_joints.AddJoint ( "J3", 2, Vector3DF(0,0,0), 1, 0, 0 );	
	m_joints.SetLimits ( 1, Vector3DF(-45, 0, 0), Vector3DF(90, 0, 0) );	 
	m_joints.SetLimits ( 2, Vector3DF(5, 0, 0), Vector3DF(170, 0, 0) );	 
	m_joints.SetLimits ( 3, Vector3DF(5, 0, 0), Vector3DF(90, 0, 0) );	 
	*/

	m_joints.StartIK();

	m_goal.Set ( 2, 1, 4 );
}

bool Sample::init ()
{	
	int w = getWidth(), h = getHeight();			// window width & height

	start_log ( "log.txt" );
	
	init2D ( "arial" );
	setview2D ( w, h );	
	glViewport ( 0, 0, w, h );

	cam = new Camera3D;
	cam->setNearFar ( 1, 2000 );
	cam->setOrbit ( Vector3DF(40,30,0), Vector3DF(0,0,0), 70, 70 );
	m_adjust = -1;

	// create joints
	Reset ();	

	m_bRunInv = true;

	return true;
}


void Sample::display()
{
	Vector3DF a,b,c,p;

	clearScreenGL();

	if (m_bRunInv)
		m_joints.InverseKinematics ( m_goal );

	// draw joints
	m_joints.Sketch( cam );

	

	start3D(cam);
	
	// sketch a grid
	for (int i = -10; i <= 10; i++) {
		drawLine3D(float(i),-0.01f, -10.f, float(i), -0.01f, 10.f, .2f, .2f, .2f, 1.f);
		drawLine3D(-10.f,	-0.01f, float(i), 10.f, -0.01f, float(i), .2f, .2f, .2f, 1.f);
	}	
	// world axes
	a = Vector3DF(1,0,0); b = Vector3DF(0,1,0); c = Vector3DF(0,0,1); 
	p = Vector3DF(-10.f,-0.01f,-10.f);
	drawLine3D ( p.x, p.y, p.z, p.x+a.x, p.y+a.y, p.z+a.z, 1, 0, 0, 1 );
	drawLine3D ( p.x, p.y, p.z, p.x+b.x, p.y+b.y, p.z+b.z, 0, 1, 0, 1 );
	drawLine3D ( p.x, p.y, p.z, p.x+c.x, p.y+c.y, p.z+c.z, 0, 0, 1, 1 );
		
	// goal
	p = m_goal;
	drawBox3D ( p.x-0.1f, p.y-0.1f, p.z-0.1f, p.x+0.1f, p.y+0.1f, p.z+0.1f, 1, 0, 1, 1 );
	drawBox3D ( p.x-0.1f, 0.f, p.z-0.1f, p.x+0.1f, 0.f, p.z+0.1f, .75f, 0, .75f, 1 );
	if ( mouse_action == MOVE_GOAL_XY )
		drawBox3D ( -10.f, 0.f, p.z-0.1f, 10.f, 8.f, p.z+0.1f, 1, 0, 1, 1 );
	if ( mouse_action == MOVE_GOAL_XZ )
		drawBox3D ( -10.f, p.y-0.1f, -10.f, 10.f, p.y+0.1f, 10.f, 1, 0, 1, 1 );

	end3D();



	// Use NvGui to draw in 2D/3D
	draw3D ();										// Render the 3D drawing groups
	drawGui (0);									// Render the GUI
	draw2D (); 

	postRedisplay();								// Post redisplay since simulation is continuous
}



// Geometry utilities --------------------------------------------------------------------------------------

#define EPS		0.00001

// Line A: p1 to p2
// Line B: p3 to p4
bool intersectLineLine ( Vector3DF p1, Vector3DF p2, Vector3DF p3, Vector3DF p4, Vector3DF& pa, Vector3DF& pb, double& mua, double& mub)
{
   Vector3DF p13,p43,p21;
   double d1343,d4321,d1321,d4343,d2121;
   double numer,denom;

   p13 = p1;	p13 -= p3;   
   p43 = p4;	p43 -= p3;
   if (fabs(p43.x) < EPS && fabs(p43.y) < EPS && fabs(p43.z) < EPS) return false;
   p21 = p2;	p21 -= p1;
   if (fabs(p21.x) < EPS && fabs(p21.y) < EPS && fabs(p21.z) < EPS) return false;

   d1343 = p13.Dot ( p43 );
   d4321 = p43.Dot ( p21 );
   d1321 = p13.Dot ( p21 );
   d4343 = p43.Dot ( p43 );
   d2121 = p21.Dot ( p21 );

   denom = d2121 * d4343 - d4321 * d4321;
   if (fabs(denom) < EPS) return false;
   numer = d1343 * d4321 - d1321 * d4343;

   mua = numer / denom;
   mub = (d1343 + d4321 * (mua)) / d4343;

   pa = p21;	pa *= (float) mua;		pa += p1;
   pb = p43;	pb *= (float) mub;		pb += p3;
   
   return true;
}

Vector3DF intersectLineLine (  Vector3DF p1, Vector3DF p2, Vector3DF p3, Vector3DF p4 )
{
	Vector3DF pa, pb;
	double ma, mb;
	if ( intersectLineLine ( p1, p2, p3, p4, pa, pb, ma, mb ) ) {
		return pa;
	}
	return p2;
}

float intersectLineBox ( Vector3DF p1, Vector3DF p2, Vector3DF bmin, Vector3DF bmax )
{
	Vector3DF p;
	Vector3DF nearp, farp;
	float t[6];
	Vector3DF dir;
	dir = p2; dir -= p1;
	
	int bst1 = -1, bst2 = -1;		// bst1 = front face hit, bst2 = back face hit

	t[0] = ( bmax.y - p1.y ) / dir.y;			// 0 = max y
	t[1] = ( bmin.x - p1.x ) / dir.x;			// 1 = min x
	t[2] = ( bmin.z - p1.z ) / dir.z;			// 2 = min z
	t[3] = ( bmax.x - p1.x ) / dir.x;			// 3 = max x
	t[4] = ( bmax.z - p1.z ) / dir.z;			// 4 = max z
	t[5] = ( bmin.y - p1.y ) / dir.y;			// 5 = min y
	
    p = dir * t[0]; p += p1;    if ( p.x < bmin.x || p.x > bmax.x || p.z < bmin.z || p.z > bmax.z ) t[0] = -1;
    p = dir * t[1]; p += p1;    if ( p.y < bmin.y || p.y > bmax.y || p.z < bmin.z || p.z > bmax.z ) t[1] = -1;
    p = dir * t[2]; p += p1;    if ( p.x < bmin.x || p.x > bmax.x || p.y < bmin.y || p.y > bmax.y ) t[2] = -1;
    p = dir * t[3]; p += p1;    if ( p.y < bmin.y || p.y > bmax.y || p.z < bmin.z || p.z > bmax.z ) t[3] = -1;
    p = dir * t[4]; p += p1;    if ( p.x < bmin.x || p.x > bmax.x || p.y < bmin.y || p.y > bmax.y ) t[4] = -1;
    p = dir * t[5]; p += p1;    if ( p.x < bmin.x || p.x > bmax.x || p.z < bmin.z || p.z > bmax.z ) t[5] = -1;

	for (int j=0; j < 6; j++) 
		if ( t[j] > 0.0 && ( t[j] < t[bst1] || bst1==-1 ) ) bst1=j;
	for (int j=0; j < 6; j++)
		if ( t[j] > 0.0 && ( t[j] < t[bst2] || bst2==-1 ) && j!=bst1 ) bst2=j;

	if ( bst1 == -1 ) 
		return 0.0f;
	
	if ( p1.x >= bmin.x && p1.y >= bmin.y && p1.z >= bmin.z && p1.x <= bmax.x && p1.y <= bmax.y && p1.z <= bmax.z ) {
		return t[bst2];
	} else {	
		return t[bst1];
	}
}

Vector3DF intersectLinePlane ( Vector3DF p1, Vector3DF p2, Vector3DF p0, Vector3DF pnorm )
{
	Vector3DF u, w;
	u = p2;	u -= p1;					// ray direction
	w = p1;	w -= p0;

    float dval = pnorm.Dot( u );
    float nval = -pnorm.Dot( w );

    if (fabs(dval) < EPS ) {			// segment is parallel to plane
        if (nval == 0) return p1;       // segment lies in plane
		else			return p1;      // no intersection
    }
    // they are not parallel, compute intersection
    float t = nval / dval;
    u *= t;
	u += p1;
    return u;
}

void Sample::motion(int x, int y, int dx, int dy) 
{
	// Get camera for GVDB Scene
	bool shift = (getMods() & NVPWindow::KMOD_SHIFT);		// Shift-key to modify light

	float fine = 0.5;

	switch ( mouse_down ) {	
	case NVPWindow::MOUSE_BUTTON_LEFT: {

		if ( mouse_action == MOVE_GOAL_XY ) {
			cam->setSize( getWidth(), getHeight()  );
			Vector3DF dir = cam->inverseRay( x, y );
			Vector3DF hit = intersectLinePlane ( cam->getPos(), cam->getPos()+dir, m_goal, Vector3DF(0,0,1) );
			m_goal = hit;
		} else {
			if ( m_adjust == -1 ) {
				// Adjust camera orbit 
				Vector3DF angs = cam->getAng();
				angs.x += dx*0.2f*fine;
				angs.y -= dy*0.2f*fine;				
				cam->setOrbit ( angs, cam->getToPos(), cam->getOrbitDist(), cam->getDolly() );				
			} else {			
				bool bUseXAxis = (fabs(dy) > 2 );
				m_joints.MoveJoint ( m_adjust, bUseXAxis ? 0 : 1, bUseXAxis ? -dy*0.01f : dx*0.01f);
			}
		}
		postRedisplay();	// Update display
		} break;
	
	case NVPWindow::MOUSE_BUTTON_MIDDLE: {
		// Adjust target pos		
		cam->moveRelative ( float(dx) * fine*cam->getOrbitDist()/1000, float(-dy) * fine*cam->getOrbitDist()/1000, 0 );	
		postRedisplay();	// Update display
		} break;
	
	case NVPWindow::MOUSE_BUTTON_RIGHT: {	

		if ( mouse_action == MOVE_GOAL_XZ ) {
			cam->setSize( getWidth(), getHeight()  );
			Vector3DF dir = cam->inverseRay( x, y );
			Vector3DF hit = intersectLinePlane ( cam->getPos(), cam->getPos()+dir, m_goal, Vector3DF(0,1,0) );
			m_goal = hit;
		} else {
			if ( m_adjust == -1 ) {			
				// Adjust dist
				float dist = cam->getOrbitDist();
				dist -= dy*fine;
				cam->setOrbit ( cam->getAng(), cam->getToPos(), dist, cam->getDolly() );		
			} else {			
				m_joints.MoveJoint ( m_adjust, 2, dx*0.01f );
			}		
		}
		postRedisplay();	// Update display
		} break;
	}
}

void Sample::mouse ( NVPWindow::MouseButton button, NVPWindow::ButtonAction state, int mods, int x, int y)
{
	if ( guiHandler ( button, state, x, y ) ) return;	
	
	mouse_down = (state == NVPWindow::BUTTON_PRESS) ? button : -1;		// Track when we are in a mouse drag

	mouse_action = 0;

	cam->setSize( getWidth(), getHeight()  );
	Vector3DF dir = cam->inverseRay( x, y );
	float t = intersectLineBox ( cam->getPos(), cam->getPos()+dir, m_goal-Vector3DF(0.1f,0.1f,0.1f), m_goal+Vector3DF(0.1f,0.1f,0.1f) );
	if ( t != 0 && state==NVPWindow::BUTTON_PRESS)
		if (button==NVPWindow::MOUSE_BUTTON_LEFT)
			mouse_action = MOVE_GOAL_XY;
		else
			mouse_action = MOVE_GOAL_XZ;
}

void Sample::keyboardchar(unsigned char key, int mods, int x, int y)
{
	switch ( key ) {
	case 's':
		m_joints.InverseKinematics ( m_goal, 1 );
		break;
	case ' ':	
		m_bRunInv = !m_bRunInv;
		break;
	case 'c': case 'C':	m_adjust = -1;	break;
	case 'r':		
		Reset ();
		break;
	case '0':		 	m_adjust = 0;	break;
	case '1':		 	m_adjust = 1;	break;
	case '2':		 	m_adjust = 2;	break;
	case '3':		 	m_adjust = 3;	break;
	};
}


void Sample::reshape (int w, int h)
{
	postRedisplay();
}

int sample_main ( int argc, const char** argv ) 
{
	Sample sample_obj;
	//int w=1280, h=1024;
	//int w=1920, h=1080;
	int w=1200, h=900;
	
	return sample_obj.run ( "Shapes", "Shapes", argc, argv, w, h, 4, 4 );
}

void sample_print( int argc, char const *argv)
{
}

