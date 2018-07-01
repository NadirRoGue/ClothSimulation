using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic physics manager capable of simulating a given ISimulable
/// implementation using diverse integration methods: explicit,
/// implicit, Verlet and semi-implicit.
/// </summary>
public class PhysicsManager : MonoBehaviour 
{
	/// <summary>
	/// Default constructor. Zero all. 
	/// </summary>
	public PhysicsManager()
	{
		this.m_simulable = null;
		this.m_isSimulable = false;

		this.Paused = true;
		this.OneStep = false;
		this.TimeStep = 0.01f;
		WindChangeIntervalSeconds = 5.0f;
		this.Wind = new Vector3 (0.0f, 0.15f, -0.6f);
		this.Gravity = new Vector3 (0.0f, -9.81f, 0.0f);
		this.IntegrationMethod = Integration.Explicit;
		this.SimulableObject = null;
	}

	/// <summary>
	/// Integration method.
	/// </summary>
	public enum Integration
	{
		Explicit = 0,
		Symplectic = 1,
		SemiImplicit = 2,
        MidPoint = 3,
        Verlet = 4
	};

	#region InEditorVariables
	
	public bool Paused;
	public bool OneStep;
	public float TimeStep;
	public float WindChangeIntervalSeconds;
	public Vector3 Gravity;
	public bool AllowRandomWindChanges;
	public Vector3 Wind;
	public GameObject SimulableObject;
	public Integration IntegrationMethod;

	#endregion

	#region OtherVariables

	private ISimulable m_simulable;

	private bool m_isSimulable;
	
	private VectorXD m_vxPre;
	private VectorXD m_vvPre;

	private float AccumulatedTimeInterval = 0.0f;
		
	#endregion

	#region MonoBehaviour

	public void Start () 
	{
		// Default initialization
		this.m_simulable = null;

		// Try to get simulable component from the object
		this.m_simulable = this.SimulableObject.GetComponent<ISimulable> ();

		// Check if component was found
		if (this.m_simulable == null) 
		{
			System.Console.WriteLine ("[ERROR] Couldn't find any ISimulable component");
			this.m_isSimulable = false;
		} 
		else 
		{
			System.Console.WriteLine ("[TRACE] Succesfully found ISimulable component");
			this.m_isSimulable = true;
		}

		if (this.m_isSimulable) 
		{
			// Initialize simulable model
			this.m_simulable.initialize ();

			// Update forces/matrices after the change
			this.m_simulable.updateForcesAndMatrices ();

			// Store current position and velocity vectors
			this.m_vxPre = this.m_simulable.getPositionVector();
			this.m_vvPre = this.m_simulable.getVelocityVector();
		}
	}

	public void Update()
	{
		if (Input.GetKeyUp (KeyCode.O))
			this.OneStep = !this.OneStep;
	
		if (Input.GetKeyUp (KeyCode.P))
			this.Paused = !this.Paused;
	}

	public void FixedUpdate () 
	{
		if (this.Paused && !this.OneStep)
			return; // Not simulating

		if (this.OneStep) // One!
			this.OneStep = false;

		// Simulate if possible
		if (this.m_isSimulable) 
		{
			// Update forces/matrices after the change
			this.m_simulable.updateForcesAndMatrices ();

			// Select integration method
			switch (this.IntegrationMethod)
			{
			case Integration.Explicit: this.stepExplicit(this.m_simulable); break;
			case Integration.Symplectic: this.stepSymplectic(this.m_simulable); break;
			case Integration.SemiImplicit: this.stepSemiImplicit(this.m_simulable); break;
			case Integration.MidPoint: this.stepMidpoint(this.m_simulable); break;
			case Integration.Verlet: this.stepVerlet(this.m_simulable); break;
			default: 
				throw new System.Exception("[ERROR] WTF? Should never happen!");
			}
			// Update simulated elements
			this.m_simulable.updateScene();
		}

		if (AllowRandomWindChanges) {
			AccumulatedTimeInterval += TimeStep;
			if (AccumulatedTimeInterval >= WindChangeIntervalSeconds) {
				AccumulatedTimeInterval = 0.0f;
				Wind.y = Mathf.Cos (Mathf.PI * Random.value) * Random.value;
				Wind.z = -Random.value;
			}

			float coefficent = Mathf.Sin (Time.realtimeSinceStartup * 5.0f) * 0.15f;
			Wind.x = coefficent;
		}
	}

	#endregion

	/// <summary>
	/// Performs a simulation step using Explicit integration.
	/// </summary>
	private void stepExplicit(ISimulable o)
	{
		o.updateForcesAndMatrices ();

		float h = TimeStep;
		VectorXD p = o.getPositionVector ();
		MatrixXD M = o.getMassMatrix ();
		VectorXD v = o.getVelocityVector ();
		VectorXD F = o.getForceVector ();

		VectorXD velocities = v + h * (F/M.Diagonal());
		o.fixSimulationVector (velocities);

		VectorXD newPositions = p + v * h;

		o.setPositionVector (newPositions);
		o.setVelocityVector (velocities);
    }

	/// <summary>
	/// Performs a simulation step using Symplectic integration.
	/// </summary>
	private void stepSymplectic(ISimulable o)
	{
		o.updateForcesAndMatrices ();

		float h = TimeStep;
		VectorXD p = o.getPositionVector ();
		MatrixXD M = o.getMassMatrix ();
		VectorXD v = o.getVelocityVector ();
		VectorXD F = o.getForceVector ();

		/*
		MatrixXD A = M + (h * D) + (h * h * K);
		VectorXD b = ((M + h * D) * v) + h * F;

		VectorXD velocities = A.Solve (b);*/

		VectorXD velocities = v + h * (F/M.Diagonal());
		o.fixSimulationVector (velocities);

		VectorXD newPositions = p + velocities * h;

		o.setPositionVector (newPositions);
		o.setVelocityVector (velocities);
    }

	/// <summary>
	/// Performs a simulation step using MidPoint integration.
	/// </summary>
	private void stepMidpoint(ISimulable o)
	{
		o.updateForcesAndMatrices ();

		float h = TimeStep;
		VectorXD p = o.getPositionVector ();
		MatrixXD M = o.getMassMatrix ();
		VectorXD v = o.getVelocityVector ();
		VectorXD F = o.getForceVector ();

		/*
		MatrixXD A = M + (h * D) + (h * h * K);
		VectorXD b = ((M + h * D) * v) + h * F;

		VectorXD velocities = A.Solve (b);*/

		VectorXD midPointVelocities = v + h/2.0f * (F/M.Diagonal());
		o.fixSimulationVector (midPointVelocities);

		VectorXD midPointPositions = p + v * h / 2.0f;

		o.setPositionVector (midPointPositions);
		o.setVelocityVector (midPointVelocities);

		o.updateForcesAndMatrices ();

		F = o.getForceVector ();

		VectorXD velocities = v + F / M.Diagonal () * h;
		VectorXD newPositions = p + midPointVelocities * h;

		o.fixSimulationVector (velocities);

		o.setPositionVector (newPositions);
		o.setVelocityVector (velocities);
	}

	/// <summary>
	/// Performs a simulation step using Verlet integration.
	/// </summary>

	private void stepVerlet(ISimulable o)
	{
		o.updateForcesAndMatrices ();

		float h = TimeStep;
		VectorXD p = o.getPositionVector ();
		MatrixXD M = o.getMassMatrix ();
		VectorXD v = o.getVelocityVector ();
		VectorXD F = o.getForceVector ();

		VectorXD a = F / M.Diagonal ();
		o.fixSimulationVector (a);
		//float pos = 2 * data.Pos - data.PrePos + a * (TimeStep * TimeStep);
		//float velocity = (pos - data.PrePos) / (2 * TimeStep);

		VectorXD newPositions = 2 * p - m_vxPre + a * h * h;
		VectorXD newVelocities = (newPositions - m_vxPre) / (2 * h);
		o.fixSimulationVector (newVelocities);

		m_vxPre = p;
		m_vvPre = v;

		o.setPositionVector (newPositions);
		o.setVelocityVector (newVelocities);
	}

	/// <summary>
	/// Performs a simulation step using Semi-Implicit integration.
	/// </summary>
	private void stepSemiImplicit(ISimulable o)
	{
		o.updateForcesAndMatrices ();

		float h = TimeStep;
		VectorXD p = o.getPositionVector ();
		MatrixXD M = o.getMassMatrix ();
		MatrixXD K = o.getSpringsJacobian ();
		MatrixXD D = o.getDampingMatrix ();
		VectorXD v = o.getVelocityVector ();
		VectorXD F = o.getForceVector ();

		MatrixXD A = M + (h * D) + (h * h * K);
		VectorXD b = ((M + h * D) * v) + h * F;

		VectorXD velocities = A.Solve (b);

		o.fixSimulationVector (velocities);

		VectorXD newPositions = p + velocities * h;

		o.setPositionVector (newPositions);
		o.setVelocityVector (velocities);
	}

}
