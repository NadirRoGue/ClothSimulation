using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;


/**
 * Class used as comparator of a map of edges 
 */
public class EdgeComparer : IEqualityComparer<int[]>
{
	private int hashLength;

	public EdgeComparer(int hl)
	{
		hashLength = hl;
	}

	public bool Equals(int[] a, int[] b)
	{
		if ((a [0] == b [0] && a [1] == b [1])
			|| (a [0] == b [1] && a [1] == b [0]))
			return true;
		return false;
	}

	public int GetHashCode(int[] a)
	{
		int top, bottom;
		if (a [0] > a [1]) {
			top = a [0];
			bottom = a [1];
		} else {
			top = a [1];
			bottom = a [0];
		}

		int hCode = top * hashLength + bottom;
		return hCode.GetHashCode();
	}
}

/// <summary>
/// Basic mass-spring model component which can be dropped onto
/// a game object and configured so that the set of nodes and
/// edges behave as a mass-spring model.
/// </summary>
public class SimulableNodesMS : MonoBehaviour, ISimulable
{
	/// <summary>
	/// Default constructor. All zero. 
	/// </summary>
	public SimulableNodesMS()
	{
		this.Manager = null;
		this.SceneNodes = new List<SceneNode> ();
		this.SceneEdges = new List<SceneEdge> ();
		this.Ke_T = 1.0f;
		this.Ke_F = 0.01f;
		this.Kd = 0.0f;
	}

	#region EditorVariables

	public PhysicsManager Manager;
	public List<SceneNode> SceneNodes;
	public List<SceneEdge> SceneEdges;

	/// <summary>
	/// Elastic stiffness.
	/// </summary>
	public float Ke_T;

	public float Ke_F;

	/// <summary>
	/// Damping stiffness.
	/// </summary>
	public float Kd;

	public float NodeMass;

	#endregion

    #region OtherVariables
	
	protected Node[] m_vnodes;
	protected Edge[] m_vedges;

    protected VectorXD m_vf;
    protected MatrixXD m_mJ;
	protected MatrixXD m_mD;
	protected MatrixXD m_mM;

    protected List<MSSpring> m_vsprings;

    #endregion

	#region MonoBehaviour

	private Transform def;

	// Nothing to do here

	#endregion

    #region ISimulable

	/*
	 * if C_DEBUG is #defined, it will be used to draw debug lines
	 * in the position of the edges
	 */ 
	private void DrawDebugLine(Vector3 a, Vector3 b, Color c)
	{
		#if C_DEBUG
		Debug.DrawLine (a, b, c, 100.0f, false);
		#endif
	}

	/*
	 * Creates a spring between to vertices (nodes)
	 */ 
	private SceneEdge CreateEdge(Transform parent, int a, int b, Vector3[] vertices, bool isTraction, SceneNode[] arrayNode)
	{
		Vector3 posA = vertices [a];
		Vector3 posB = vertices [b];

		Vector3 AB = posB - posA;
		float magnitude = AB.magnitude;
		AB.Normalize ();

		GameObject go = new GameObject ();
		go.name = "Edge_" + a + "_" + b;
		go.transform.parent = parent;
		go.transform.localPosition = posA + (AB * (magnitude / 2.0f));

		SceneEdge ab = go.AddComponent<SceneEdge>();
		ab.isTraction = true;
		ab.SceneNode0 = arrayNode [a];
		ab.SceneNode1 = arrayNode [b];

		#if C_DEBUG
		DrawDebugLine (transform.position + vertices [a], transform.position + vertices [b], isTraction? Color.red : Color.green);
		#endif

		return ab;
	}

	/**
	 * Parses the mesh component into a mass-spring system
	 */ 
	private void parseMesh()
	{
		def = transform.Find("default");
		MeshFilter mf = def.GetComponent<MeshFilter> ();
		Mesh mesh = mf.mesh;

		int[] triangles = mesh.triangles;
		Vector3[] vertices = mesh.vertices;

		List<SceneNode> nodes = new List<SceneNode> ();
		SceneNode[] arrayNode = new SceneNode[vertices.Length];

		GameObject nodesParent = new GameObject ();
		nodesParent.name = "Nodes";
		nodesParent.transform.parent = def;
		nodesParent.transform.localPosition = Vector3.zero;

		GameObject topLeftCorner = null;
		GameObject topRightCorner = null;

		float lowerY = float.PositiveInfinity;
		float higherY = float.NegativeInfinity;
		float rightMostX = float.NegativeInfinity;

		for(int i = 0; i < vertices.Length; i++) {

			Vector3 v = vertices [i];

			// Create new scene node
			GameObject empty = new GameObject();

			empty.transform.parent = nodesParent.transform;

			empty.name = "Node_" + i;
			empty.transform.localPosition = v;

			#if C_DEBUG
			GameObject sphere = GameObject.CreatePrimitive (PrimitiveType.Sphere);
			sphere.transform.parent = empty.transform;
			sphere.transform.localScale = new Vector3 (0.1f, 0.1f, 0.1f);
			sphere.transform.localPosition = Vector3.zero;
			#endif

			SceneNode sn = empty.AddComponent<SceneNode> ();
			sn.Mass = NodeMass;
			sn.IsFixed = false;
			SceneNodeWrapper snw = empty.AddComponent<SceneNodeWrapper> ();
			snw.gObject = def;
			snw.vertexId = i;

			nodes.Add (sn);
			arrayNode [i] = sn;

			if (v.y <= lowerY) {
				if (v.z >= rightMostX) {
					rightMostX = v.z;
					lowerY = v.y;
					topLeftCorner = empty;
				}
			}
			if (v.y >= higherY) {
				if (v.z >= rightMostX) {
					rightMostX = v.z;
					higherY = v.y;
					topRightCorner = empty;
				}
			}
		}

		topLeftCorner.GetComponent<SceneNode>().IsFixed = true;
		topRightCorner.GetComponent<SceneNode>().IsFixed = true;

		GameObject edgesObj = new GameObject ();
		edgesObj.name = "Edges";
		edgesObj.transform.parent = def;
		edgesObj.transform.localPosition = Vector3.zero;

		SceneEdges = new List<SceneEdge> ();
		Dictionary<int[], int> sharedEdges = new Dictionary<int[], int> (new EdgeComparer(vertices.Length));
		for (int i = 0; i < triangles.Length; i = i + 3) {
			int a = triangles [i];
			int b = triangles [i + 1];
			int c = triangles [i + 2];

			// Flexion nodes
			int[] aEdge = new int[]{ b, c };
			int[] bEdge = new int[]{ c, a };
			int[] cEdge = new int[]{ a, b };

			// A
			int [] key;
			if (sharedEdges.ContainsKey ((key = aEdge)) || sharedEdges.ContainsKey ((key = new int[]{ c, b }))) {
				int other = sharedEdges [key];
				SceneEdge oa = CreateEdge (edgesObj.transform, other, a, vertices, false, arrayNode);
				SceneEdges.Add (oa);
				sharedEdges.Remove (key);
			} else {
				sharedEdges.Add (aEdge, a);
				SceneEdge bc = CreateEdge (edgesObj.transform, b, c, vertices, true, arrayNode);
				SceneEdges.Add (bc);
			}

			// B
			if (sharedEdges.ContainsKey ((key = bEdge)) || sharedEdges.ContainsKey ((key = new int[]{ a, c }))) {
				int other = sharedEdges [key];
				SceneEdge ob = CreateEdge (edgesObj.transform, other, b, vertices, false, arrayNode);
				SceneEdges.Add (ob);
				sharedEdges.Remove(key);
			} else {
				sharedEdges.Add (bEdge, b);
				SceneEdge ca = CreateEdge (edgesObj.transform, c, a, vertices, true, arrayNode);
				SceneEdges.Add (ca);

			}

			// C
			if (sharedEdges.ContainsKey ((key = cEdge)) || sharedEdges.ContainsKey ((key = new int[]{ b, a }))) {
				int other = sharedEdges [key];
				SceneEdge oc = CreateEdge (edgesObj.transform, other, c, vertices, false, arrayNode);
				SceneEdges.Add (oc);
				sharedEdges.Remove (key);
			} else {
				sharedEdges.Add (cEdge, c);
				SceneEdge ab = CreateEdge(edgesObj.transform, a, b, vertices, true, arrayNode);
				SceneEdges.Add (ab);
			}
		}

		SceneNodes = new List<SceneNode>(arrayNode);
	}

	public void initialize ()
	{
		parseMesh ();

		int numNodes = this.SceneNodes.Count;
		int numEdges = this.SceneEdges.Count;
		this.m_vnodes = new Node[numNodes];
		this.m_vedges = new Edge[numEdges];
		
		// Start scene nodes/edges

		for (int i = 0; i < this.SceneEdges.Count; ++i)
			this.SceneEdges [i].initialize (); // Prepare
		
		for (int i = 0; i < this.SceneNodes.Count; ++i)
			this.SceneNodes [i].initialize (); // Prepare
		
		// Cache internal nodes
		for (int i = 0; i < numNodes; ++i) 
		{
			this.m_vnodes [i] = this.SceneNodes [i].Node;
			
			// Configure nodes indices
			this.m_vnodes [i].SimIdx = i;
		}
		
		// Cache internal edges
		for (int i = 0; i < numEdges; ++i)
			this.m_vedges [i] = this.SceneEdges [i].Edge;
		
		int N = this.getNumDOF ();
		this.m_vf = new DenseVectorXD (N);
		this.m_mJ = new DenseMatrixXD (N, N);
		this.m_mD = new DenseMatrixXD (N, N);
		this.m_mM = new DenseMatrixXD (N, N);

		foreach (Node n in this.m_vnodes) {
			int nodeIndex = n.SimIdx * 3;
			for (int j = 0; j < 3; j++) {
				m_mM [nodeIndex + j, nodeIndex + j] = n.Mass;
			}
		}

		// Create energy elements vector and initialize rest-state
		this.m_vsprings = new List<MSSpring> (this.m_vedges.Length);
		
		for (int i = 0; i < this.m_vedges.Length; ++i) 
		{
			MSSpring spring = new MSSpring();
			spring.Edge = this.m_vedges[i];
			if (spring.Edge.isTraction)
				spring.Ke = this.Ke_T;
			else
				spring.Ke = this.Ke_F;
			spring.L0 = 0.0f;
			
			// Restart rest and add
			spring.computeRestState();
			this.m_vsprings.Add(spring);
		}
		
		// Initialize forces and matrices
		this.clearForcesAndMatrices ();
	}

	public int getNumDOF ()
	{
		return 3 * this.m_vnodes.Length;
	}

	public int getNumNodes ()
	{
		return this.m_vnodes.Length;
	}
	
	public Node getNode (int i)
	{
		return this.m_vnodes [i];
	}
	
	public VectorXD getForceVector ()
	{
		return this.m_vf.Clone();
	}

	public VectorXD getMassVector ()
	{
		VectorXD vmout = new DenseVectorXD (this.getNumDOF ());
		for (int i = 0; i < this.getNumNodes(); ++i)
			for (int j = 0; j < 3; ++j) // Set X,Y,Z values
				vmout [3 * i + j] = this.m_vnodes[i].Mass;

		return vmout;
	}

	public VectorXD getVelocityVector ()
	{
		VectorXD vvout = new DenseVectorXD (this.getNumDOF ());

		for (int i = 0; i < this.getNumNodes(); ++i) {
			vvout [3 * i + 0] = this.m_vnodes [i].Vt [0];
			vvout [3 * i + 1] = this.m_vnodes [i].Vt [1];
			vvout [3 * i + 2] = this.m_vnodes [i].Vt [2];
		}
            
		return vvout;
	}

	public VectorXD getPositionVector ()
	{
		VectorXD vxout = new DenseVectorXD (this.getNumDOF ());

		for (int i = 0; i < this.getNumNodes(); ++i) {
			vxout [3 * i + 0] = this.m_vnodes [i].Xt [0];
			vxout [3 * i + 1] = this.m_vnodes [i].Xt [1];
			vxout [3 * i + 2] = this.m_vnodes [i].Xt [2];
		}

		return vxout;
	}

	public void setVelocityVector (VectorXD vvin)
	{
		for (int i = 0; i < this.getNumNodes(); ++i) {
			this.m_vnodes [i].Vt = new Vector3(
				(float)vvin [3 * i + 0],
				(float)vvin [3 * i + 1],
				(float)vvin [3 * i + 2]);
		}
	}
	
	public void setPositionVector (VectorXD vxin)
	{
		for (int i = 0; i < this.getNumNodes(); ++i) {
			this.m_vnodes [i].Xt = new Vector3(
				(float)vxin [3 * i + 0],
				(float)vxin [3 * i + 1],
				(float)vxin [3 * i + 2]);
		}
	}

	public MatrixXD getMassMatrix ()
	{
		return this.m_mM;
	}

	public MatrixXD getDampingMatrix ()
	{
		return this.m_mD;
	}

	public MatrixXD getSpringsJacobian ()
	{
		return this.m_mJ;
	}

	public void updateForcesAndMatrices ()
	{
		def.GetComponent<MeshFilter> ().mesh.RecalculateNormals ();
		this.clearForcesAndMatrices ();

		foreach (MSSpring spring in this.m_vsprings) {
			Node a = spring.Edge.Node1;
			Node b = spring.Edge.Node0;

			spring.addForce (m_vf);

			spring.addJacobian (m_mJ);
		}

		foreach(Node n in m_vnodes)
		{
			int index = n.SimIdx * 3;

			Vector3 normal = def.GetComponent<MeshFilter> ().mesh.normals [n.SimIdx];

			//float scale = 1.0f - Vector3.Dot(normal, Manager.Wind);
			Vector3 NodeForce = -Kd * n.Vt + Manager.Gravity * n.Mass + Manager.Wind;// * scale;
			m_vf [index] += NodeForce.x;
			m_vf [index + 1] += NodeForce.y;
			m_vf [index + 2] += NodeForce.z;

			for (int i = 0; i < 3; i++) {
				m_mD [index + i, index + i] += -Kd * n.Vt[i];
				//m_mM [index + i, index + i] = n.Mass;
			}
		}
	}

	public void clearForcesAndMatrices ()
	{
		this.m_vf.Clear ();
		this.m_mJ.Clear ();
		this.m_mD.Clear ();
		//this.m_mM.Clear ();
	}

	public void updateScene ()
	{
		// Update nodes
		for (int i = 0; i < this.SceneNodes.Count; ++i)
			this.SceneNodes [i].updateScene (); // Async.

		// Update edges
		for (int i = 0; i < this.SceneEdges.Count; ++i)
			this.SceneEdges [i].updateScene (); // Async.
	}

	public void fixSimulationVector(VectorXD v)
	{
		for (int i = 0; i < this.getNumNodes(); ++i) 
		{
			if (this.m_vnodes [i].IsFixed) 
			{
				int nodeOffset = 3*this.m_vnodes[i].SimIdx;
				for (int j = 0; j < 3; ++j)
					v[nodeOffset + j] = 0.0;
			}
		}
	}

	public void fixSimulationMatrix(MatrixXD m)
	{
		for (int i = 0; i < this.getNumNodes(); ++i)
		{
			if (this.m_vnodes[i].IsFixed)
			{			
				int nodeOffset = 3*this.m_vnodes[i].SimIdx;
				for (int j = 0; j < 3; ++j)
				{
					for (int k = 0; k < this.getNumDOF(); ++k)
					{
						m[nodeOffset + j,k] = 0.0;
						m[k,nodeOffset + j] = 0.0;
					}
					m[nodeOffset + j,nodeOffset + j] = 1.0;
				}
			}
		}
	}

    #endregion

	#region OtherMethods

	#endregion

}
