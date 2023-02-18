#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    std::vector<Vector2D> intermediate_points = {};
    Vector2D temp;
    for (int i = 0; i < points.size(); i++) {
        // do something with myVec[i]
        if (i == 0)
        {
            temp = points[i];
            continue;
        }
        else
        {
            Vector2D new_points = (1 - t) * temp + t * points[i];
            intermediate_points.push_back(new_points);
            temp = points[i];
        }
    }
    return intermediate_points;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      // totally the same as the last function
      std::vector<Vector3D> intermediate_points = {};
      Vector3D temp;
      for (int i = 0; i < points.size(); i++) {
          if (i == 0)
          {
              temp = points[i];
              continue;
          }
          else
          {
              Vector3D new_points = (1 - t) * temp + t * points[i];
              intermediate_points.push_back(new_points);
              temp = points[i];
          }
      }
      return intermediate_points;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      std::vector<Vector3D> intermediate_points = points;
      Vector3D res;
      while (evaluateStep(intermediate_points, t).size() > 1)
      {
          intermediate_points = evaluateStep(intermediate_points, t);
      }
      intermediate_points = evaluateStep(intermediate_points, t);
      res = intermediate_points[0];
    return res;
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
      std::vector<Vector3D> intermediate_points = {};
      int n = controlPoints.size(); // get size (n) for our points
      for (int i = 0; i < n; i++)
      {
          intermediate_points.push_back(evaluate1D(controlPoints[i], u));
      }

    return evaluate1D(intermediate_points, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
      Vector3D res;
      Vector3D position = this->position;
      HalfedgeCIter h = this -> halfedge();      // get the outgoing half-edge of the vertex
      Vector3D last_position;
      Vector3D vector1;
      Vector3D vector2;
      Vector3D face_normal;
      float area;
      auto get_area = [&](Vector3D p1, Vector3D p2, Vector3D p3) {
          // a function that calculated the area of a mesh given by three position
          Vector3D line1 = p2 - p1;
          Vector3D line2 = p3 - p2;
          Vector3D line3 = p1 - p3;
          float line1_length = sqrt(pow(line1.x,2) + pow(line1.y, 2) + pow(line1.z, 2));
          float line2_length = sqrt(pow(line2.x, 2) + pow(line2.y, 2) + pow(line2.z, 2));
          float line3_length = sqrt(pow(line3.x, 2) + pow(line3.y, 2) + pow(line3.z, 2));
          float area;
          float p = (line1_length + line2_length + line3_length);
          area = sqrt(p * (p - line1_length) * (p - line2_length) * (p - line3_length));
          return area;
      };

      auto normalize = [&](Vector3D p) {
          // a normalize function
          float line_length = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));
          p = p / line_length;
          return p;
      };

      do {
          HalfedgeCIter h_twin = h -> twin(); // get the opposite half-edge
          VertexCIter v = h_twin -> vertex(); // vertex is the 'source' of the half-edge, so
          if (h == this->halfedge())
          {
              last_position = v->position;
          }
          else
          {
              vector1 = position - last_position;
              vector2 = v->position - last_position;
              face_normal = cross(vector1, vector2);
              area = get_area(position, last_position, v->position);
              res = res + area * face_normal;
              last_position = v->position;

          }
          Vector3D last_position = v->position;
          // h->vertex() is v, whereas h_twin->vertex()
          // is the neighboring vertex
          //cout << v -> position << endl;      // print the vertex position
          h = h_twin -> next();               // move to the next outgoing half-edge of the vertex
      } while (h != this->halfedge());          // keep going until we are back where we were
    return normalize(res);
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
      HalfedgeIter h1 = e0->halfedge();
      if (h1->isBoundary())
      {
          return e0;
      }
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h2->next();
      HalfedgeIter h4 = h1->twin();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h5->next();
      //Vertices
      VertexIter a = h1->vertex();
      VertexIter b = h2->vertex();
      VertexIter d = h3->vertex();
      VertexIter c = h6->vertex();
      //Faces
      FaceIter f0 = h1->face(); 
      FaceIter f1 = h4->face(); 
      //Edges
      EdgeIter bd = h2->edge();
      EdgeIter da = h3->edge();
      EdgeIter ac = h5->edge();
      EdgeIter cb = h6->edge();

      h1->setNeighbors(h3, h4, c, e0, f0);
      h2->setNeighbors(h4, h2->twin(), b, bd, f1);
      h3->setNeighbors(h5, h3->twin(), d, da, f0);
      h4->setNeighbors(h6, h1, d, e0, f1);
      h5->setNeighbors(h1, h5->twin(), a, ac, f0);
      h6->setNeighbors(h2, h6->twin(), c, cb, f1);
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
      a->halfedge() = h5;
      b->halfedge() = h2;
      c->halfedge() = h6;
      d->halfedge() = h3;

      e0->halfedge() = h1;
      bd->halfedge() = h2;
      da->halfedge() = h3;
      ac->halfedge() = h5;
      cb->halfedge() = h6;

      f0->halfedge() = h1;
      f1->halfedge() = h4;
      e0 = h1->edge();
     
    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
      VertexIter m  = newVertex();
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      HalfedgeIter h1 = e0->halfedge();
      if (h1->isBoundary())
      {
          return m;
      }
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h2->next();
      HalfedgeIter h4 = h1->twin();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h5->next();
      //Vertices
      VertexIter a = h1->vertex();
      VertexIter b = h2->vertex();
      VertexIter d = h3->vertex();
      VertexIter c = h6->vertex();
      //Faces
      FaceIter f0 = h1->face();
      FaceIter f1 = h4->face();
      //Edges
      EdgeIter bd = h2->edge();
      EdgeIter da = h3->edge();
      EdgeIter ac = h5->edge();
      EdgeIter cb = h6->edge();

      // following is what we did
      Vector3D p1 = a->position;
      Vector3D p2 = b->position;
      m->position = (p1 + p2) / 2;
      HalfedgeIter h7 = newHalfedge();
      HalfedgeIter h8 = newHalfedge();
      HalfedgeIter h9 = newHalfedge();
      HalfedgeIter h10 = newHalfedge();
      HalfedgeIter h11 = newHalfedge();
      HalfedgeIter h12 = newHalfedge();
      FaceIter f2 = newFace();
      FaceIter f3 = newFace();
      //EdgeIter ma = newEdge(); e0 = ma!
      EdgeIter mb = newEdge();
      EdgeIter mc = newEdge();
      EdgeIter md = newEdge();
      h1->setNeighbors(h2, h4, m, mb, f1);
      h2->setNeighbors(h7, h2->twin(), b, bd, f1);
      h3->setNeighbors(h9, h3->twin(), d, da, f2);
      h4->setNeighbors(h12, h1, b, mb, f0);
      h5->setNeighbors(h11, h5->twin(), a, ac, f3);
      h6->setNeighbors(h4, h6->twin(), c, cb, f0);
      h7->setNeighbors(h1, h8, d, md, f1);
      h8->setNeighbors(h3, h7, m, md, f2);
      h9->setNeighbors(h8, h10, a, e0, f2);
      h10->setNeighbors(h5, h9, m, e0, f3);
      h11->setNeighbors(h10, h12, c, mc, f3);
      h12->setNeighbors(h6, h11, m, mc, f0);
      // assign to vertex
      m->halfedge() = h1;
      a->halfedge() = h5;
      b->halfedge() = h2;
      c->halfedge() = h6;
      d->halfedge() = h3;
      // assign to edge
      bd->halfedge() = h2;
      da->halfedge() = h3;
      ac->halfedge() = h5;
      cb->halfedge() = h6;
      e0->halfedge() = h10;
      mb->halfedge() = h1;
      mc->halfedge() = h12;
      md->halfedge() = h8;
      // assign the face
      f0->halfedge() = h4;
      f1->halfedge() = h1;
      f2->halfedge() = h3;
      f3->halfedge() = h5;

      // new judge
      m->isNew = true;
      e0->isNew = false;
      mc->isNew = true;
      mb->isNew = false;
      md->isNew = true;

    return m;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    
    // 4. Flip any new edge that connects an old and new vertex.

    // 5. Copy the new vertex positions into final Vertex::position.
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          v->isNew = false;
          int n = v->degree();
          double u = (n == 3) ? 0.1875 : 0.375 / n;
          v->newPosition = v->position * (1. - n * u);
          HalfedgeCIter h = v->halfedge();
          do {
              v->newPosition += u * (h->next()->vertex()->position);
              h = h->twin()->next();
          } while (h != v->halfedge());

      }

      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
          HalfedgeCIter h = e->halfedge();
          e->newPosition =
              0.375 * h->vertex()->position +
              0.375 * h->twin()->vertex()->position +
              0.125 * h->next()->next()->vertex()->position +
              0.125 * h->twin()->next()->next()->vertex()->position;
      }

      int nEdges = mesh.nEdges();
      EdgeIter e = mesh.edgesBegin();
      for (int i = 0; i < nEdges; i++) {
          VertexIter m = mesh.splitEdge(e);
          m->newPosition = e->newPosition;
          m->isNew = true;
          e++;
      }

      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
          VertexIter v1 = e->halfedge()->vertex();
          VertexIter v2 = e->halfedge()->twin()->vertex();
          if (e->isNew && (v1->isNew != v2->isNew)) {
              mesh.flipEdge(e);
          }
      }

      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          v->position = v->newPosition;
      }

  }
}
