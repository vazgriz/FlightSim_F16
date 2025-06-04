using System;
using UnityEngine;

public static class Table {
	/// <summary>
	/// Reads a 1 dimensional table with a non-zero starting index
	/// </summary>
	/// <param name="table">The table</param>
	/// <param name="i">The index</param>
	/// <param name="start">The starting index</param>
	/// <returns></returns>
	public static float ReadTable(float[] table, int i, int start) {
		return table[i - start];
	}

	/// <summary>
	/// Reads a 2-dimensional table with non-zero starting index
	/// </summary>
	/// <param name="table">The table</param>
	/// <param name="x">The X index</param>
	/// <param name="y">The Y index</param>
	/// <param name="xStart">The starting X index</param>
	/// <param name="yStart">The starting Y index</param>
	/// <returns></returns>
	public static float ReadTable(float[,] table, int x, int y, int xStart, int yStart) {
		return table[y - yStart, x - xStart];
	}

	/// <summary>
	/// Calculates indices for a lookup table. Allows interpolation beyond the ends of the table
	/// min and max should be set to 1 index within the table bounds
	/// So if a table has bounds [0, 5), min and max should be 1 and 3 respectively
	/// K0 will be within [min, max]
	/// K1 may be within [min - 1, max + 1]
	/// T is an interpolation value that says how far beyond table the input value is
	/// </summary>
	/// <param name="value">The input value</param>
	/// <param name="scale">The scaling factor of input</param>
	/// <param name="min">The minimum K0 value</param>
	/// <param name="max">The maximum K0 value</param>
	/// <returns></returns>
	public static (int k0, int k1, float t) GetLookUpIndex(float value, float scale, int min, int max) {
		float scaled = value * scale;
		int K0 = Mathf.Clamp((int)scaled, min, max);
		float T = scaled - K0;
		int K1 = K0 + (int)Mathf.Sign(T);

		return (K0, K1, T);
	}

	public static float LinearLookup(float value, float scale, float[] table, int min, int max) {
		(int k0, int k1, float kT) = GetLookUpIndex(value, scale, min + 1, max - 1);
		float T = ReadTable(table, k0, min);
		float U = ReadTable(table, k1, min);
		float result = T + Math.Abs(kT) * (U - T);
		return result;
	}

	public static float BilinearLookup(float xValue, float xScale, float yValue, float yScale, float[,] table, int xMin, int xMax, int yMin, int yMax) {
		(int x0, int x1, float xT) = GetLookUpIndex(xValue, xScale, xMin + 1, xMax - 1);
		(int y0, int y1, float yT) = GetLookUpIndex(yValue, yScale, yMin + 1, yMax - 1);
		float T = ReadTable(table, x0, y0, xMin, yMin);
		float U = ReadTable(table, x0, y1, xMin, yMin);
		float V = T + Math.Abs(xT) * (ReadTable(table, x1, y0, xMin, yMin) - T);
		float W = U + Math.Abs(xT) * (ReadTable(table, x1, y1, xMin, yMin) - U);

		float result = V + (W - V) * Math.Abs(yT);
		return result;
	}
}