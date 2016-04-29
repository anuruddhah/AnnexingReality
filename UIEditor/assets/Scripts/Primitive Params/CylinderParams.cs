using UnityEngine;
using System.Collections;
using System;

namespace AnnexR{
	namespace Primitives{
		public class CylinderParams : BaseParams{

			public CylinderParams(){
				radiusPref = 0.5f;
				heightPref = 0.3f;
				ratioPref = 0.2f;
			}

			public float radius;
			public float height;

			public float radiusMin;
			public float radiusMax;
			public float heightMin;
			public float heightMax;

			public Vector3 axisPosition;
			public Vector3 axisDirection;

			// To avoid random rotations around the main axis
			public Vector3 localRight;
			public Vector3 localForward;
			
			public float radiusPref;
			public float heightPref;
			public float ratioPref;
		};
	}
}