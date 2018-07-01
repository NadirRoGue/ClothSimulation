using UnityEngine;
using System.Collections;
using System.Collections.Generic;

/// <summary>
/// Basic class providing a wrapper component for a simulation node. 
/// This should be responsible of telling Unity to update the visual
/// representation of the node, as well as allowing the user to change 
/// properties from the editor (e.g. IsFixed).
/// </summary>
public class SceneNode : MonoBehaviour
{
	/// <summary>
	/// Default constructor. All zero. 
	/// </summary>
	public SceneNode ()
	{
		this.Node = new Node ();

		SceneEdges = new List<SceneEdge> ();
	}

	#region InEditorVariables
	
	public float Mass = 1.0f;

	public bool IsFixed = false;
	
	#endregion 

	#region NoEditorProperties

	public Node Node { get; set; }

	public List<SceneEdge> SceneEdges { get; set; }

	#endregion
	
	#region FromMonoBehaviour
	
	// Nothing to do here
	
	#endregion
	
	#region OtherMethods

	/// <summary>
	/// Initialize the node from scene. 
	/// </summary>
	public void initialize ()
	{
		// Initialize from scene
		this.Node.Mass = this.Mass;
		if (IsFixed)
			Debug.Log ("isFixed");
		this.Node.IsFixed = this.IsFixed;
		this.Node.X0 = this.transform.position;
		this.Node.Xt = this.transform.position;
		this.Node.Vt.Set (0.0f, 0.0f, 0.0f);
	}

	/// <summary>
	/// Updates the scene representation of this edge.
	/// </summary>
	public void updateScene ()
	{
		// Update scene/editor
		this.Mass = this.Node.Mass;
		this.IsFixed = this.Node.IsFixed;
		this.transform.position = this.Node.Xt;
	}
	
	#endregion
}
