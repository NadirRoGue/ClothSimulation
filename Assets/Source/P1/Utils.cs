using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic class for common operations.
/// </summary>
public class Utils
{
    /// <summary>
    /// Convert a Vector3 to VectorXD
    /// </summary>
    public static VectorXD ToVectorXD(Vector3 vin)
    {
        VectorXD vout = new DenseVectorXD(3);
        vout[0] = vin[0];
        vout[1] = vin[1];
        vout[2] = vin[2];

        return vout;
    }

	/// <summary>
	/// Convert a VectorXD to Vector3
	/// </summary>
    public static Vector3 ToVector3(VectorXD vin)
    {
        Vector3 vout = new Vector3();
        vout.Set((float) vin[0], 
                 (float) vin[1], 
                 (float) vin[2]);
        return vout;
    }

	/// <summary>
	/// Add the specified 3D vector to the concatenaded vector.
	/// </summary>
	public static void Add3D(int idx, VectorXD v3, VectorXD v)
	{
		v.SetSubVector (3 * idx, 3, v3 + v.SubVector(3 * idx, 3));
	}

	/// <summary>
	/// Add the specified 3x3 matrix to the specified matrix.
	/// </summary>
	public static void Add3D(int idx0, int idx1, MatrixXD m3, MatrixXD m)
	{
		m.SetSubMatrix (3 * idx0, 3, 3 * idx1, 3, m3 + m.SubMatrix (3 * idx0, 3, 3 * idx1, 3));
	}

}
