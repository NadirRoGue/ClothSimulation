using UnityEngine;
using System.Collections;
using System.Collections.Generic;

/// <summary>
/// Basic class enclosing the dynamic and structural information of an edge.
/// </summary>
public class Edge 
{
	public bool isTraction;

	/// <summary>
	/// Default constructor. Initialize node instances.
	/// </summary>
	public Edge()
	{
		this.Node0 = null;
		this.Node1 = null;
	}

	/// <summary>
	/// Node at one extreme.
	/// </summary>
	public Node Node0 { get; set; }

	/// <summary>
	/// Node at other extreme.
	/// </summary>
	public Node Node1 { get; set; }
	
}
