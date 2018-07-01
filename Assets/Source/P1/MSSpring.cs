using UnityEngine;
using System.Collections;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;


/// <summary>
/// Elastic energy component corresponding to a simple spring.
/// </summary>
public class MSSpring 
{
	/// <summary>
	/// Default constructor. All zero.
	/// </summary>
	public MSSpring()
	{
		this.Edge = null;
		this.Ke = -1.0f;
		this.L0 = -1.0f;
	}

	public Edge Edge { get; set; }

    public float Ke { get; set; }

    public float L0 { get; set; }

    /// <summary>
    /// Computes the data dependent on the rest state 
    /// of the spring. In this case, only rest-length.
    /// </summary>
    public void computeRestState()
    {
        Node node0 = Edge.Node0;
        Node node1 = Edge.Node1;
        L0 = (node1.X0 - node0.X0).magnitude;
    }

    /// <summary>
    /// Returns the energy of the spring at current state.
    /// </summary>
    public float getEnergy()
    {
        Node node0 = Edge.Node0;
        Node node1 = Edge.Node1;
        float L = (node1.Xt - node0.Xt).magnitude;
        return 0.5f * this.Ke * (L - L0)*(L - L0);
    }

    /// <summary>
    /// Adds the elastic forces corresponding to the string
    /// to the specified global forces vector at the index
	/// associated with edge nodes. Note |vfsum| = nDOF.
    /// </summary>
    public void addForce(VectorXD vfsum)
    {
		int index1 = Edge.Node1.SimIdx * 3;
		int index0 = Edge.Node0.SimIdx * 3;

		Vector3 LR3 = (Edge.Node1.Xt - Edge.Node0.Xt);
		float deltaL = LR3.magnitude - L0;
		Vector3 dir = LR3.normalized;

		Vector3 springForce = -Ke * deltaL * dir;

		vfsum [index1] += springForce.x;
		vfsum [index1 + 1] += springForce.y;
		vfsum [index1 + 2] += springForce.z;

		vfsum [index0] += -springForce.x;
		vfsum [index0 + 1] += -springForce.y;
		vfsum [index0 + 2] += -springForce.z;
		// TO COMPLETE: Exercise 2
    }

	/// <summary>
	/// Adds the elastic Jacobian corresponding to the string
	/// to the specified global Jacobian matrix at the index
	/// associated with edge nodes. Note |mJSum| = (nDOF,nDOF).
	/// </summary>
	public void addJacobian(MatrixXD mJsum)
	{
		Node a = Edge.Node1;
		Node b = Edge.Node0;

		int index1 = a.SimIdx * 3;
		int index0 = b.SimIdx * 3;

		Vector3 LR3 = a.Xt - b.Xt;
		float L = LR3.magnitude;

		float kLL0 = Ke * (L - L0);
		Vector3 U = LR3.normalized;
		MatrixXD I = DenseMatrixXD.CreateIdentity (3);

		MatrixXD uut = DenseMatrixXD.Create (3, 3, 0.0);
		for (int i = 0; i < 3; i++) {
			for(int j = 0; j < 3; j++) {
				uut [i, j] = U [i] * U [j];
			}
		}

		MatrixXD dUdxa = 1.0f / L * (I - uut);

		MatrixXD dFadxa = kLL0 * dUdxa + Ke * uut;

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {

				mJsum [index1 + i, index1 + j] += dFadxa[i,j];
				mJsum [index0 + i, index0 + j] += dFadxa[i,j];

				mJsum [index1 + i, index0 + j] += -dFadxa[i,j];
				mJsum [index0 + i, index1 + j] += -dFadxa[i,j];
			}
		}
	}
}
