using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic interface for any simulation model.
/// </summary>
public interface ISimulable
{
	/// <summary>
	/// Initialize the simulable.
	/// </summary>
	void initialize();

	/// <summary>
	/// Returns the number of model DOF.
	/// </summary>
	int getNumDOF();

	/// <summary>
	/// Returns the number of model nodes.
	/// </summary>
    int getNumNodes();

	/// <summary>
	/// Returns the i-th node of the model.
	/// </summary>
    Node getNode(int i);

	/// <summary>
	/// Returns the whole force vector |f| = nDOF.
	/// </summary>
	VectorXD getForceVector();

    /// <summary>
	/// Returns the whole velocity vector |v| = nDOF.
    /// </summary>
    VectorXD getVelocityVector();

    /// <summary>
    /// Returns the whole position vector |x| = nDOF.
    /// </summary>
    VectorXD getPositionVector();

	/// <summary>
	/// Returns the whole lumped mass vector |x| = nDOF.
	/// </summary>
	VectorXD getMassVector();
	
	/// <summary>
	/// Updates the velocity for all the nodes |x| = nDOF.
	/// </summary>
	void setVelocityVector(VectorXD vvin);
	
	/// <summary>
	/// Updates the position for all the nodes |x| = nDOF.
	/// </summary>
	void setPositionVector(VectorXD vxin);

    /// <summary>
    /// Returns the whole mass matrix.
    /// </summary>
    MatrixXD getMassMatrix();

	/// <summary>
	/// Returns the whole damping matrix.
	/// </summary>
	MatrixXD getDampingMatrix();

	/// <summary>
	/// Returns the whole Jacobian matrix.
	/// </summary>
	MatrixXD getSpringsJacobian();
	
	/// <summary>
	/// Updates model forces/Jacobian.
	/// </summary>
	void updateForcesAndMatrices();

	/// <summary>
	/// Clears model forces/Jacobian.
	/// </summary>
	void clearForcesAndMatrices();

	/// <summary>
	/// Updates the scene.
	/// </summary>
	void updateScene();

	/// <summary>
	/// Fixs the simulation vector considering the fixed
	/// nodes of the system. This method overwrites with
	/// 0's those fixed degrees of freedom.
	/// </summary>
	void fixSimulationVector(VectorXD v);

	/// <summary>
	/// Fixs the simulation matrix considering the fixed
	/// nodes of the system. This method overwrites with
	/// 0's the columns and rows of the linear system at
	/// fixed DOF, and puts 1's in the diagonal. This is
	/// valid as long as the right hand side of the LS is
	/// also zero at fixed DOF. 
	/// </summary>
	void fixSimulationMatrix(MatrixXD m);

}
