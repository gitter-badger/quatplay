#include "util.h"

bool verbose = true;
int loggingLevel = 3;

#include "ConfigReader.h"
#include <vector>
#include <map>
#include <math.h>

struct V3 { float x,y,z; };
//V2 operator - ( V2 a, V2 b ) {
	//V2 c;
	//c.x = b.x - a.x;
	//c.y = b.y - a.y;
	//return c;
//}
struct Quat {
	float s,i,j,k;
};
Quat V3ToQuat( const V3 in ) {
	Quat q = { 0.0f, in.x, in.y, in.z };
	return q;
}
V3 QuatToV3( const Quat in ) {
	V3 v = { in.i, in.j, in.k };
	return v;
}
Quat QuatFromAngleAxis( float angle, float x, float y, float z ) {
	float s = sin(angle*0.5f);
	float c = cos(angle*0.5f);
	Quat q = { c, s*x, s*y, s*z };
	return q;
}
Quat operator*( const Quat a, const Quat b ) {
	Quat q;
	q.s = a.s*b.s - a.i*b.i - a.j*b.j - a.k*b.k;
	q.i = a.s*b.i + a.i*b.s - a.k*b.j + a.j*b.k;
	q.j = a.s*b.j + a.j*b.s - a.i*b.k + a.k*b.i;
	q.k = a.s*b.k + a.k*b.s - a.j*b.i + a.i*b.j;
	return q;
}
Quat conj( const Quat in ) {
	Quat q = { in.s, -in.i, -in.j, -in.k };
	return q;
}

struct Workspace {
	std::vector<V3> verts;
	std::vector<V3> results;
	Quat quat;
};

void AddVertex( const char *line, void *user ) {
	V3 v;
	sscanf( line, "%f,%f,%f", &v.x, &v.y, &v.z );
	Workspace *w = (Workspace*)user;
	w->verts.push_back( v );
}
void AddResult( const char *line, void *user ) {
	V3 v;
	sscanf( line, "%f,%f,%f", &v.x, &v.y, &v.z );
	Workspace *w = (Workspace*)user;
	w->results.push_back( v );
}
void SetupQuaternion( const char *line, void *user ) {
	Quat q;
	int num = sscanf( line, "%f,%f,%f,%f", &q.s, &q.i, &q.j, &q.k );
	if( num == 4 ) {
		Workspace *w = (Workspace*)user;
		w->quat = q;
		logf( 1, "Load Quaternion %.2f, %.2f,%.2f,%.2f\n", q.s, q.i,q.j,q.k );
		return;
	}
	num = sscanf( line, "%f*%f,%f,%f", &q.s, &q.i, &q.j, &q.k );
	if( num == 4 ) {
		Workspace *w = (Workspace*)user;
		logf( 1, "Load AxisAngle %.2f (%.2fpi), %.2f,%.2f,%.2f\n", q.s, q.s/M_PI, q.i,q.j,q.k );
		q = QuatFromAngleAxis(q.s,q.i,q.j,q.k);
		w->quat = q;
		return;
	}
}
LineReaderCallback GetCallbackFromHeader( const char *line, void * ) {
	if( 0 == strcasecmp( line, "verts" ) ) { return AddVertex; }
	if( 0 == strcasecmp( line, "results" ) ) { return AddResult; }
	if( 0 == strcasecmp( line, "quat" ) ) { return SetupQuaternion; }
	logf( 1, "Unexpected config header [%s], returning null handler\n", line );
	return 0;
}


#define TestAssert( X ) if( !( X ) ) { logf( 1, RED "TEST FAILED" CLEAR "[%s]\n", #X ); return false; } else { logf( 3, GREEN "TEST PASSED" CLEAR "[%s]\n", #X ); }

bool TestQuat( int testID ) {
	char filename[128];
	sprintf( filename, "test%i.txt", testID );
	Workspace w;
	int result = OpenConfigAndCallbackPerLine( filename, GetCallbackFromHeader, 0, &w );
	TestAssert( 0 == result );
	TestAssert( w.verts.size() > 0 );
	TestAssert( w.results.size() > 0 );
	//logf( 1, "Testing %i vs %i\n", w.verts.size(), w.results.size() );
	TestAssert( w.results.size() == w.verts.size() );
	Quat q = w.quat;
	Quat iq = conj(w.quat);
	logf( 1, "Testing multiplication by Q(%.2f,%.2f,%.2f,%.2f)\n", q.s,q.i,q.j,q.k );
	for( size_t i = 0; i < w.verts.size(); ++i ) {
		V3 in = w.verts[i];
		V3 result = w.results[i];
		logf( 1, "Testing Q(%.2f,%.2f,%.2f,%.2f) * V(%.2f,%.2f,%.2f) * Q(%.2f,%.2f,%.2f,%.2f) = V(%.2f,%.2f,%.2f)\n", q.s,q.i,q.j,q.k, in.x,in.y,in.z, iq.s,iq.i,iq.j,iq.k, result.x, result.y,result.z );
		Quat r = V3ToQuat( in );
		logf( 1, "r = %f,%f,%f,%f\n", r.s,r.i,r.j,r.k );
		Quat qr = q * r;
		logf( 1, "qr = %f,%f,%f,%f\n", qr.s,qr.i,qr.j,qr.k );
		Quat out = qr * iq;
		logf( 1, "out = %f,%f,%f,%f\n", out.s,out.i,out.j,out.k );
		V3 v = QuatToV3( out );
		logf( 1, "v = %f,%f,%f\n", v.x, v.y, v.z );
		TestAssert( 0.1 > fabs( v.x - result.x ) );
		TestAssert( 0.1 > fabs( v.y - result.y ) );
		TestAssert( 0.1 > fabs( v.z - result.z ) );
	}
	return true;
}

int main( ) {
	TestAssert( 1 == 1 );

	for( int tid = 1; tid <= 5; ++tid ) {
		TestAssert( TestQuat(tid) );
	}

	logf( 1, GREEN "ALL TESTS PASSED" CLEAR "\n" );

	return 0;
}
