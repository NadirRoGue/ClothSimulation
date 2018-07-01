using UnityEngine;
using System.Collections;
using System.Collections.Generic;

/// <summary>
/// Basic class providing a wrapper component for a simulation edge. 
/// This should be responsible of telling Unity to update the visual
/// representation of the edge, as well as allowing the user to change 
/// properties from the editor (e.g. SceneNode0 and SceneNode1).
/// </summary>
public class SceneEdge : MonoBehaviour
{
	/// <summary>
	/// Default constructor. All zero. 
	/// </summary>
	public SceneEdge ()
	{
		this.Edge = new Edge ();
	}

	#region InEditorVariables

	public bool IsFixed = false;
	public bool isTraction = false;
	
	public SceneNode SceneNode0 = null;
	public SceneNode SceneNode1 = null;
	
	#endregion 
	
	#region NoEditorPropereties
	
	public Edge Edge { get; set; }

	#endregion

	#region MonoBehaviour
	
	// Nothing to do here
	
	#endregion
	
	#region OtherMethods

	/// <summary>
	/// Initialize the edge from scene. 
	/// </summary>
	public void initialize ()
	{
		if (SceneNode0 == null)
			throw new System.Exception ("[ERROR] SceneNode0 must be initialized");
		
		if (SceneNode1 == null)
			throw new System.Exception ("[ERROR] SceneNode1 must be initialized");
		
		// Add cross-reference to scene nodes
		
		this.SceneNode0.SceneEdges.Add (this);
		this.SceneNode1.SceneEdges.Add (this);
		
		// Add nodes to edge
		
		this.Edge.Node0 = this.SceneNode0.Node;
		this.Edge.Node1 = this.SceneNode1.Node;

		this.Edge.isTraction = isTraction;
		
		// Add cross-reference to nodes
		
		this.SceneNode0.Node.Edges.Add(this.Edge);
		this.SceneNode1.Node.Edges.Add(this.Edge);
		
		// Fix both nodes if necessary
		
		if (IsFixed) 
		{
			this.Edge.Node0.IsFixed = true;
			this.Edge.Node1.IsFixed = true;
		}
	}

	/// <summary>
	/// Updates the scene representation of this edge.
	/// </summary>
	public void updateScene ()
	{
		// Update scene nodes first
		this.SceneNode0.updateScene ();
		this.SceneNode1.updateScene ();

		// Orient and scale properly 
		Vector3 createRot = new Vector3(0.0f, 1.0f, 0.0f);
		Vector3 springRot = SceneNode1.transform.position -
							SceneNode0.transform.position;
		
		float springScale = springRot.magnitude;
		springRot = springRot * (1.0f / springScale);
		// Cylinders in Unity dimensions by default are height = 2 and radius = 0.5: scale
		transform.localPosition = (SceneNode0.transform.localPosition + SceneNode1.transform.localPosition) * 0.5f;
		transform.localScale = new Vector3(transform.localScale.x, springScale*0.5f, transform.localScale.z);
		transform.rotation = Quaternion.FromToRotation(createRot, springRot);
	}
	
	#endregion
	
}
