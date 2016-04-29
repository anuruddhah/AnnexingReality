using UnityEngine;
using System.Collections;
using System;

namespace AnnexR{
	namespace Primitives{
		public class PlaneParams : BaseParams{
			public float width;
			public float length;

			public float widthMin;
			public float widthMax;
			public float lengthMin;
			public float lengthMax;

			public Vector3 center;
			public Vector3 normal;

			// To avoid random rotations around the main axis
			public Vector3 localRight;
			public Vector3 localUp;
			
			public float widthPref;
			public float lengthPref;
			public float ratioPref;
		};
	}
}