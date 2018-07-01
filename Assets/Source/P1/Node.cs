using UnityEngine;
using System.Collections;
using System.Collections.Generic;

/// <summary>
/// Basic class enclosing the kinematic and structural information of a node.
/// </summary>
public class Node 
{
	/// <summary>
	/// Default constructor. All zero.
	/// </summary>
	public Node()
	{
		this.Mass = 0.0f;
		this.IsFixed = false;
		this.X0.Set (0.0f, 0.0f, 0.0f);
		this.Xt.Set (0.0f, 0.0f, 0.0f);
		this.Vt.Set (0.0f, 0.0f, 0.0f);
		this.SimIdx = -1;

		this.Edges = new List<Edge> ();
	}

	/// <summary>
	/// The mass of the node. 
	/// </summary>
	public float Mass { get; set; }

	/// <summary>
	/// The rest position.
	/// </summary>
	public Vector3 X0 { get; set; }

	/// <summary>
	/// Current position.
	/// </summary>
	public Vector3 Xt { get; set; }

	/// <summary>
	/// Current velocity.
	/// </summary>
	public Vector3 Vt { get; set; }

	/// <summary>
	/// Node simulation index.
	/// </summary>
    public int SimIdx { get; set; }

	/// <summary>
	/// Is the node fixed?
	/// </summary>
	public bool IsFixed { get; set; }

	/// <summary>
	/// List of node edges.
	/// </summary>
	public List<Edge> Edges { get; set; }

}
