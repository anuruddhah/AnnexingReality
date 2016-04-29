using UnityEngine;
using System.Collections;
using System;

namespace AnnexR{
	namespace Primitives{
		public class SphereParams : BaseParams{

			public SphereParams(){
				radiusPref = 1.0f;
			}

			public float radius;

			public float radiusMin;
			public float radiusMax;

			public Vector3 center;

			// To avoid random rotations around the main axis
			public Vector3 localRight;
			public Vector3 localUp;
			public Vector3 localForward;
			
			public float radiusPref;
		};
	}
}