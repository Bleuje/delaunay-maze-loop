#pragma once

#include "ofMain.h"

#include <string>
#include <gmp.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Delaunay_mesh_face_base_2<K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds> CDT;
typedef CGAL::Delaunay_mesh_size_criteria_2<CDT> Criteria;
typedef CGAL::Delaunay_mesher_2<CDT, Criteria> Meshing_engine;
typedef CDT::Vertex_handle Vertex_handle;
typedef CDT::Face_handle Face_handle;
typedef CDT::Point Point;
typedef K::Point_2 Point_2;

class ofApp : public ofBaseApp{
    CDT cdt; // the constrained delaunay triangulation

    int n = 50; // number of points to define the boundary
    float R = 1.0; // Parameter for size of the shape
    float scl = 300; // factor to get positions in pixels

    std::map<Vertex_handle, int> V; // vertex_handle -> int index
    std::map<int, Vertex_handle> V_rev; // int index -> vetex_handle
    std::map<Vertex_handle, bool> isOnBoundary;
    std::map<Vertex_handle, bool> visited;
    int N; // number of vertices of the triangulation

    // building the tree
    Vertex_handle startVertex;
    std::vector<std::vector<Vertex_handle> > adj;
    std::vector<std::vector<Vertex_handle> > adj2;
    void buildTree();

    //building the curve around the tree
    std::vector<ofVec2f> curvePoints;
    void buildCurve();
    void manageVertex(const Vertex_handle& v, const Vertex_handle& prev);

    int NPoints; // number of points of the curve

    std::vector<double> lengths;
    std::vector<double> cumLengthSums;

    ofVec2f pointPos(Vertex_handle v,Vertex_handle v1,Vertex_handle v2);
    void buildLengths();
    //float pSegment = 0.0;
    ofVec2f curvePos(float p);
    ofVec2f smoothCurvePos(float p); // curve parametrization with p from 0 to 1

    int numFrames = 40; // number of gif frames
    int m = 600; // number of drawn objects

    ofFbo fbo;

public:
    void setup();
    void update();
    void draw();

    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);

};

