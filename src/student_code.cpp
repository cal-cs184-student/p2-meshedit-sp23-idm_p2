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
      int n = points.size();
      float t = this->t;
      std::vector<Vector2D> ret;
      for (int i = 0; i < n - 1; i++) {
          ret.push_back((1 - t) * points[i] + t * points[i + 1]);
      }
      return ret;
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
      int n = points.size();
      std::vector<Vector3D> ret;
      for (int i = 0; i < n - 1; i++) {
          ret.push_back((1 - t) * points[i] + t * points[i + 1]);
      }
      return ret;
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
      int n = points.size();
      std::vector<Vector3D> ret = points;
      for (int i = 0; i < n - 1; i++) {
          ret = evaluateStep(ret, t);
      }
      return ret[0];
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
      int n = controlPoints.size();
      std::vector<Vector3D> P;

      for (auto row: controlPoints) {
          P.push_back(evaluate1D(row, u));
      }

      return evaluate1D(P, v);
  }

  Vector3D Vertex::normal(void) const
  {
      // TODO Part 3.
      // Returns an approximate unit normal at this vertex, computed by
      // taking the area-weighted average of the normals of neighboring
      // triangles, then normalizing.
      Vector3D n(0, 0, 0);
      double totalArea = 0.;
      HalfedgeCIter h = halfedge();

      auto doublearea = [](HalfedgeCIter h) {
          Vector3D a = h->vertex()->position;
          Vector3D b = h->next()->vertex()->position;
          Vector3D c = h->next()->next()->vertex()->position;
          Vector3D x = cross(b - a, c - a);
          return x.norm();
      };
      do {
          double area = doublearea(h);
          n += h->face()->normal() / area;
          h = h->twin()->next();
      } while (h != halfedge());
      n.normalize();
      return n;
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
      HalfedgeIter h1 = e0->halfedge();
      HalfedgeIter h4 = h1->twin();
      FaceIter A = h1->face();
      FaceIter B = h4->face();
      if (A->isBoundary() || B->isBoundary()) {
          return e0;
      }

      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h2->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h5->next();

      VertexIter a = h1->vertex();
      VertexIter b = h4->vertex();
      VertexIter c = h3->vertex();
      VertexIter d = h6->vertex();

      a->halfedge() = h5;
      b->halfedge() = h2;
      c->halfedge() = h1;
      d->halfedge() = h4;

      A->halfedge() = h1;
      B->halfedge() = h4;

      h1->next() = h6;
      h6->next() = h2;
      h2->next() = h1;
      h3->next() = h5;
      h5->next() = h4;
      h4->next() = h3;

      h1->vertex() = c;
      h4->vertex() = d;

      h6->face() = A;
      h3->face() = B;
      
      return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
      // TODO Part 5.
      // This method should split the given edge and return an iterator to the newly inserted vertex.
      // The halfedge of this vertex should point along the edge that was split, rather than the new edges.

      // get existing things
      HalfedgeIter h1 = e0->halfedge();
      
      HalfedgeIter h4 = h1->twin();
      

      FaceIter A = h1->face();
      FaceIter B = h4->face();

      if (A->isBoundary() || B->isBoundary()) {
          return VertexIter();
      }

      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h2->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h5->next();

      VertexIter a = h1->vertex();
      VertexIter b = h4->vertex();
      VertexIter c = h3->vertex();
      VertexIter d = h6->vertex();


      // create new things
      VertexIter m = newVertex();
      m->position = 0.5 * a->position + 0.5 * b->position;

      HalfedgeIter h7 = newHalfedge();
      HalfedgeIter h8 = newHalfedge();
      HalfedgeIter h9 = newHalfedge();
      HalfedgeIter h10 = newHalfedge();
      HalfedgeIter h11 = newHalfedge();
      HalfedgeIter h12 = newHalfedge();

      EdgeIter e1 = newEdge();
      EdgeIter e2 = newEdge();
      EdgeIter e3 = newEdge();

      FaceIter C = newFace();
      FaceIter D = newFace();

      // start to modify
      m->halfedge() = h7;
      e0->halfedge() = h1;
      e1->halfedge() = h4;
      e2->halfedge() = h11;
      e3->halfedge() = h10;
      A->halfedge() = h1;
      B->halfedge() = h4;
      C->halfedge() = h7;
      D->halfedge() = h8;

      h1->twin() = h8;
      h1->next() = h10;

      h2->next() = h9;
      h2->face() = C;

      h4->twin() = h7;
      h4->next() = h11;
      h4->edge() = e1;

      h5->next() = h12;
      h5->face() = D;

      h7->twin() = h4;
      h7->next() = h2;
      h7->vertex() = m;
      h7->edge() = e1;
      h7->face() = C;

      h8->twin() = h1;
      h8->next() = h5;
      h8->vertex() = m;
      h8->edge() = e0;
      h8->face() = D;

      h9->twin() = h10;
      h9->next() = h7;
      h9->vertex() = c;
      h9->edge() = e3;
      h9->face() = C;

      h10->twin() = h9;
      h10->next() = h3;
      h10->vertex() = m;
      h10->edge() = e3;
      h10->face() = A;

      h11->twin() = h12;
      h11->next() = h6;
      h11->vertex() = m;
      h11->edge() = e2;
      h11->face() = B;

      h12->twin() = h11;
      h12->next() = h8;
      h12->vertex() = d;
      h12->edge() = e2;
      h12->face() = D;

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

  }
}
