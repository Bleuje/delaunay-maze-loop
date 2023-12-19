#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    Vertex_handle boundaryVertices[n];

    // Heart shaped boundary for triangulation
    for(int i=0;i<n;i++)
    {
        double theta = 2*PI*i/n;
        double R2 = R/18.0;
        double y = -R2*(13.0*cos(theta) - 5.0*cos(2.0*theta) - 2.0*cos(3*theta) - cos(4*theta));
        double x = 16*R2*sin(theta)*sin(theta)*sin(theta);
        boundaryVertices[i] = cdt.insert(Point(x, y));
    }

    for(int i=0;i<n;i++)
    {
       cdt.insert_constraint(boundaryVertices[i], boundaryVertices[(i+1)%n]);
    }

    // making the mesh from the boundary constraint
    CGAL::refine_Delaunay_mesh_2(cdt,Criteria(0.125, 0.12));

    // data structures for vertex_handle -> int index and int index -> vertex_handle
    V.clear();
    int num = 0;
    for(CDT::Finite_vertices_iterator it=cdt.finite_vertices_begin();it!=cdt.finite_vertices_end(); it++)
    {
        V[it] = num;
        V_rev[num] = it;
        num++;
    }
    N = num; // number of vertices in triangulation

    // marking vertices on boundary and also making them as visited for the tree construction
    // also making the adjacency lists defining the graph of the triangulation
    isOnBoundary.clear();
    visited.clear();
    adj.clear();
    adj = std::vector<std::vector<Vertex_handle> >(N, std::vector<Vertex_handle>());
    for(CDT::Finite_edges_iterator eit = cdt.finite_edges_begin(); eit != cdt.finite_edges_end(); ++eit)
    {
        Vertex_handle v1 = eit->first->vertex((eit->second+1)%3);
        Vertex_handle v2 = eit->first->vertex((eit->second+2)%3);
        if(cdt.is_constrained(*eit))
        {

            isOnBoundary[v1] = true;
            isOnBoundary[v2] = true;
            visited[v1] = true;
            visited[v2] = true;
        }
        adj[V[v1]].push_back(v2);
        adj[V[v2]].push_back(v1);
    }

    // building all data for the animation
    buildTree();
    buildCurve();
    buildLengths();

    //rendering settings
    ofEnableSmoothing();

    // using an fbo is not necessary, it was an experiment
    ofFbo::Settings settings;
    settings.numSamples = 8;
    settings.useDepth = true;
    settings.width = ofGetWidth();
    settings.height = ofGetHeight();

    fbo.allocate(settings);
}

// function to make a random tree
void ofApp::buildTree()
{
    // find a startVertex that's not on boundary
    while(true)
    {
        int ind = ofRandom(0,0.99999*N);
        Vertex_handle v = V_rev[ind];
        if(!isOnBoundary[v])
        {
            startVertex = v;
            break;
        }
    }

    // making the tree similarly to DFS or BFS but with a priority queue instead of a stack or queue
    // tree graph (adjacency lists) is stored in adj2
    adj2.clear();
    adj2 = std::vector<std::vector<Vertex_handle> >(N, std::vector<Vertex_handle>());
    std::priority_queue<std::pair<float, Vertex_handle> > pq; // using a priority queue with weights to select randomly the next node vertex to explore from the list
    pq.push({0.5,startVertex});
    visited[startVertex] = true;
    while(!pq.empty())
    {
        Vertex_handle v = pq.top().second;
        pq.pop();
        for(Vertex_handle u : adj[V[v]])
        {
            if(!visited[u])
            {
                visited[u] = true;
                pq.push({ofRandom(0,1),u});
                adj2[V[v]].push_back(u);
                adj2[V[u]].push_back(v);
            }
        }
    }
}

// here we're rotating around v, looking at v1 and v2 and we want a point position of the triangle (v,v1,v2) close to v
ofVec2f ofApp::pointPos(Vertex_handle v,Vertex_handle v1,Vertex_handle v2)
{
    float weightTowardsV = 1.5;
    return ofVec2f((v1->point().x() + v2->point().x() + (1.0+weightTowardsV)*v->point().x())/(3.0+weightTowardsV),(v1->point().y() + v2->point().y() + (1.0+weightTowardsV)*v->point().y())/(3.0+weightTowardsV));
}

// recursive function to construct a curve around the tree
void ofApp::manageVertex(const Vertex_handle& v,const Vertex_handle& prev)
{
    std::vector<Vertex_handle> vlist;
    int ind = 0; // the index of the vertex we come from
    int num = 0; // current index around v
    int sz = 0; // number of vertices around v
    CDT::Vertex_circulator circ = cdt.incident_vertices(v), done0(circ);
      do {sz++;} while(++circ != done0);

    // making the list of vertices around v, and finding out the index of the vertex we come from
    CDT::Vertex_circulator circulator = cdt.incident_vertices(v), done(circulator);
      do
        {
          if(circulator==prev) ind = num;
          vlist.push_back(circulator);
          num++;
        } while(++circulator != done);

    num = ind; // we start rotating from ind, the previous vertex
    bool start = true;

    while(start||num!=ind)
    {
        start = false;
        // recursive call if we're looking towards an edge of the tree (in adj2) that's not the edge we come from
        if(std::find(adj2[V[v]].begin(), adj2[V[v]].end(), vlist[num]) != adj2[V[v]].end() && vlist[num]!=prev) {
            if(vlist[num]!=startVertex) manageVertex(vlist[num], v);
        }

        // add point to the curve
        ofVec2f pt = pointPos(v,vlist[num],vlist[(num+1)%sz]);
        curvePoints.push_back(pt);

        num = (num+1)%sz;
    }
}

void ofApp::buildCurve()
{
    curvePoints.clear();

    manageVertex(startVertex, startVertex);
}

// data structures of the curve
void ofApp::buildLengths()
{
    lengths.clear();
    cumLengthSums.clear();
    cumLengthSums.push_back(0);

    NPoints = curvePoints.size();

    double curSum = 0; // current sum of lengths

    for(int i=0;i<NPoints;i++)
    {
        ofVec2f p1 = curvePoints[i];
        ofVec2f p2 = curvePoints[(i+1)%NPoints];
        double len = ofDist(p1.x,p1.y,0,p2.x,p2.y,0);
        lengths.push_back(len);
        curSum += len;
        cumLengthSums.push_back(curSum);
     }
}

// idea for parametrization with constant speed : use binary search on cumulative sums array
ofVec2f ofApp::curvePos(float p)
{
    float p2 = std::fmod(p+2020,1.0);
    double totalLength = cumLengthSums[NPoints];

    float q = p2*0.9999999*totalLength; // 0.9999 probably useless, just to be sure to avoid reaching totalLength

    // binary search
    std::vector<double>::iterator it = std::upper_bound(cumLengthSums.begin(),cumLengthSums.end(),q);

    int ind2 = it - cumLengthSums.begin();
    int ind1 = ind2-1;

    float rem = q-cumLengthSums[ind1];
    float lp = ofMap(rem,0,lengths[ind1],0,1);

    ofVec2f v1 = curvePoints[ind1];
    ofVec2f v2 = curvePoints[ind2%NPoints];

    return ofVec2f(ofLerp(v1.x,v2.x,lp),ofLerp(v1.y,v2.y,lp));
}

// trying to get smoother curve corners with an average
ofVec2f ofApp::smoothCurvePos(float p)
{
    float eps = 0.2/m;
    ofVec2f v1 = curvePos(p-eps/2);
    ofVec2f v2 = curvePos(p+eps/2);

    return ofVec2f((v1.x+v2.x)/2.0,(v1.y+v2.y)/2.0);
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
    fbo.begin();
    ofBackground(0);

    ofPushMatrix();
    ofTranslate(fbo.getWidth()/2,fbo.getHeight()/2);

    //float t = 1.0*ofGetMouseX()/ofGetWidth();
    float t = 1.0*ofGetFrameNum()/numFrames;

    ofSetColor(255);
    ofFill();

    for(int i=0;i<m;i++)
    {
        int K = 13;
        for(int j=0;j<K;j++)
        {
            float p = 1.0*(i+t+0.045*j)/m;

            float sz = 0.3+5.0*pow(1.0*(j+1)/K,1.75);

            ofVec2f pos = smoothCurvePos(p);

            ofDrawEllipse(pos.x*scl,pos.y*scl,sz,sz);
        }
    }

    ofPopMatrix();
    fbo.end();

    fbo.draw(0,0, ofGetWidth(), ofGetHeight());

    // Saving frames
    if(ofGetFrameNum()<numFrames){
        std::ostringstream str;
        int num = ofGetFrameNum();
        std::cout << "Saving frame " << num << "\n" << std::flush;
        str << std::setw(3) << std::setfill('0') << num;
        ofSaveScreen("fr"+str.str()+".gif");
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}
