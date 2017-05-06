//-------------------------------------------------------------------------
// This is an attempt at implementing Adrian Sfarti's tessellation patent
//-------------------------------------------------------------------------

// -------------------------------------
// Constant Buffers
//--------------------------------------
// This object is available at all points in the pipeline:

cbuffer cbPerFrame : register(b0)
{
    matrix   g_mView;             // The view-transformation
    matrix   g_mProj;             // The projection-transformation
    float3   g_vCameraPosWorld;   // The world position of the Camera

    // The THRESHOLD we compare flatness against
    float    g_fTessellationThreshold;   

    // Actually a BOOLEAN:  Should we pay attention to control-points weights?
    float    g_bNurbs;

    // Near and Far plane distances
    float    g_nearPlane;
    float    g_farPlane;
    float    padding[1];
};

// The input patch size.  In this sample, it is 16 control points.
// This value should match the call to IASetPrimitiveTopology()
#define INPUT_PATCH_SIZE 16

// The output patch size.  In this sample, it is also 16 control points.
#define OUTPUT_PATCH_SIZE 16


//--------------------------------------------------------------------------------------
// Vertex shader section
//--------------------------------------------------------------------------------------
struct VS_CONTROL_POINT_INPUT
{
    float4 vPosition : POSITION;
};

struct VS_CONTROL_POINT_OUTPUT
{
    float4 vPosition : POSITION;
};

// This simple vertex shader passes the control points straight through to the
// hull shader.  In a more complex scene, you might transform the control points
// or perform skinning at this step.
// The input to the vertex shader comes from the vertex buffer.
// The output from the vertex shader will go into the hull shader.

VS_CONTROL_POINT_OUTPUT BezierVS(VS_CONTROL_POINT_INPUT Input)
{
    VS_CONTROL_POINT_OUTPUT Output;
    Output.vPosition = Input.vPosition;
    return Output;
}


// ###################################################
// HULL SHADER
// ###################################################


// -----------------------------------------------
// Geometry Utilities for the HULL SHADER:
//------------------------------------------------

// Returns the distance between the point p,
// and the line that contains s and e.
float distance(float3 p, float3 s, float3 e) {
    float3 v = p - s;
    float3 d = e - s;
    float3 n = d / length(d);
    return length(v - n * dot(v, n));
}

// Midpoint between two points:
float3 midpoint(float3 a, float3 b) {
    return (a + b) / 2;
}

// -------------------------------------
// BEZIER CURVES in 3D SPACE
//--------------------------------------

// A Curve is defined by 4 points in 3D space
struct Curve {
    float3 p1, p2, p3, p4;
};

// This is the FLATNESS MEASURE:
// Note this will work right ONLY if the Curve points are in "CAMERA SPACE",
// which means the Camera is at the origin looking along the z axis:
float getFlatness(Curve c) {
    return (max(distance(c.p2, c.p1, c.p4), distance(c.p3, c.p1, c.p4))
           * 2 * g_nearPlane / (c.p2.z + c.p3.z));
}

// This is the result of SUBDIVIDING a Bezier Curve to get two curves:
struct TwoCurves {
    Curve c1;
    Curve c2;
};

// This is the code that SUBDIVIDES a Bezier Curve to get two curves:
TwoCurves subdivide(Curve c) {
    float3 P1 = c.p1;
    float3 P2 = c.p2;
    float3 P3 = c.p3;
    float3 P4 = c.p4;

    // Refer to Figure 5 of the 501 patent:
    float3 L2    = midpoint(P1, P2);
    float3 H     = midpoint(P2, P3);
    float3 R3    = midpoint(P3, P4);
    float3 L3    = midpoint(L2, H);
    float3 R2    = midpoint(H, R3);
    float3 L4_R1 = midpoint(L3, R2);

    TwoCurves result;

    result.c1.p1 = P1;
    result.c1.p2 = L2;
    result.c1.p3 = L3;
    result.c1.p4 = L4_R1;

    result.c2.p1 = L4_R1;
    result.c2.p2 = R2;
    result.c2.p3 = R3;
    result.c2.p4 = P4;

    return result;
}

// -----------------------------------------------------
// Split Positions BIT-VECTOR
// -----------------------------------------------------

// By subdividing a Bezier Curve until a flatness threshold is exceeded, 
// the Hull Shader determines at what <s> and <t> values the patch should split.
// The split values will be fractions with a power-of-two denominator.
//
// For example, if the top-level curve exceeds the threshold, a split
// is introduced at 0.5 and two child curves are created.  If the first
// child curve exceeds the threshold but the second doesn't, this means
// a split is needed at 0.25 but not needed at 0.75.   If the new child
// curves are flat enough so the subdivision stops at this point,
// then in this example, we have a total of two splits: 0.25 and 0.5.
//
// However, the "fixed function tessellation stage", when told to split
// the unit square into N pieces, will generate internal vertices at
// evenly positioned locations, each a multiple of 1/N.   If told to
// split a patch into 3 pieces, (as in the previous example),
// the "fixed function tessellation stage" will generate internal vertices
// with coordinates of 0.33 and 0.66.   It will be up to the Domain Shader
// to "map" these evenly-spaced coordinates to the arbitrarily positioned
// ones determined in the Hull Shader.

#define NUM_BITS 32

struct Divisions {
    uint splits;
};

// "Divisions" is a bit-vector, here containing 32 bits.
// This is how the Hull Shader represents what split values
// are desired for a given coordinate.
//
// Each bit represents a 1/32nd portion of the unit interval,
// specifically, whether there is a split to the right of it.
//
// The leftmost bit (bit #31) represents the coordinate at 1/32.
// If the bit is set, it means a split should occur at 1/32.
//
// The next leftmost bit (bit #30) represents 2/32, and so on,
// (in general bit "i" represents the value (NUM_BITS-i)/NUM_BITS).
// A split at 0.25, discussed above, would correspond to bit 24.
// The split at 0.5 would correspond to bit 16.
//
// Here are some functions that set or read specific bits:

Divisions setBit(Divisions d, int bit) {
    d.splits |= (1 << bit);
    return d;
}
int getBit(Divisions d, int bit) {
    return ((d.splits & (1 << bit)) != 0) ? 1 : 0;
}

Divisions emptyDivisions() {
    Divisions d;
    d.splits = 0;
    return d;
}

// Count how many bits are set in a Divisions.
// This becomes the "tessellation factor".

int getNumSetBits(Divisions d) {
    int c = 0;
    for (int i = 0; i < NUM_BITS; ++i) {
        if (getBit(d, i) == 1) c++;
    }
    return c;
}

// The functions below are used in the Domain Shader, which needs to "map"
// the coordinates received from the "fixed function tessellation stage".
//
// In the example above, (where two splits at 0.25 and 0.50 are desired),
// the "tessellation factor" would be three, so the coordinates received
// from the "fixed function tessellation stage" would be 0.33 and 0.66.
// To get the desired splitting, the Domain Shader will have to transform
// the coordinates such that 0.33 becomes 0.25, and 0.66 becomes 0.5.

int nextSetBit(Divisions d, int bit) {
    while ((--bit > 0) && (getBit(d, bit) == 0)) {}
    return bit;
}
float reshift(Divisions d, int numSetBits, float val) {
    float nth = 1.0f / numSetBits;
    float mth = 1.0f / NUM_BITS;
    int bit = NUM_BITS;
    float m = 0;
    for (int i = 0; i < numSetBits; ++i) {
        bit = nextSetBit(d, bit);
        float om = m;
        m = NUM_BITS - bit;

        if (val < (1 + i) * nth) {
            return (om + (m - om) * ((val - i * nth) / nth)) * mth;
        }
    }
    return 1.0f;
}

// -----------------------------------------------------
// SELECTING Split Positions
// by recursively subdividing a Bezier Curve
// -----------------------------------------------------

// While a recursive function is the most natural way to
// express recursive-subdivision, it produced a DirectX error
// complaining that Hull Shader code cannot be recursive.  (!!??!)
//
// So, a "queue" is used instead,
// to do the same work without recursion.
//
// Below, a "Range" can represent the top-level curve or 
// any of the child curves that result from subdivision,
// associated with the coordinate-range the curve represents.

struct Range {
    Curve c;
    uint leftBit, rightBit;
};
Divisions addSubdivisions(Curve c, Divisions d2) {
    Range ranges[NUM_BITS];
    int numR = 0;

    Divisions d = d2;

    // The Curve "c" is associated with the full 0.0 - 1.0 interval, 
    // that is, bit 31 through bit 0 inclusive:
    Range r;
    r.c = c;
    r.leftBit = NUM_BITS - 1;
    r.rightBit = 0;
    ranges[numR++] = r;

    while (numR > 0) {
        Range r1 = ranges[--numR];

        // Each subdivision causes a BIT to be set in the "Divisions"
        // bitvector.  After all ranges are processed, the bits that are set
        // correspond to the locations at which splits should occur.
        d = setBit(d, r1.rightBit);

        // If "rightBit" is not less than "leftBit" then they must be equal,
        // which means the "recursive" subdivision has reached a maximum depth.
        // Using a 32-bit bitvector, the smallest interval (1/32nd in length)
        // is reached after 5 successive subdivisions.
        //
        // But if "rightBit" is strictly less than "leftBit", then our width
        // is two (1/32) intervals or wider, and it's possible to subdivide:
        if (r1.rightBit < r1.leftBit) {

            // COMPUTE FLATNESS:
            float flatness = getFlatness(r1.c);

            // SHOULD we subdivide?
            if (flatness > g_fTessellationThreshold) {

                // YES -- we're going to subdivide the curve!
                TwoCurves div = subdivide(r1.c);

                // Both child curves are added to the queue
                // for later testing:
                uint midPoint = (r1.leftBit + r1.rightBit + 1) / 2;

                r.leftBit = r1.leftBit;
                r.rightBit = midPoint;
                r.c = div.c1;
                ranges[numR++] = r;

                r.leftBit = midPoint-1;
                r.rightBit = r1.rightBit;
                r.c = div.c2;
                ranges[numR++] = r;
            }
        }
    }
    return d;
}

// ----------------------------------------------------------
// The "Hull Shader" main entry-point
// ----------------------------------------------------------

// The hull shader has to output 16 control points.
// Each control point will be one of these HS_OUTPUT objects.
// It's a point in space, plus a WEIGHT that's
// used only if the patch is a NURBS patch:

struct HS_OUTPUT
{
    float3 vPosition  : BEZIERPOS;
    float weight : BLENDWEIGHT;
};

// This extracts one control point from the "InputPatch" structure.
HS_OUTPUT fromInputPatch(InputPatch<VS_CONTROL_POINT_OUTPUT, INPUT_PATCH_SIZE> p, int i) {
    HS_OUTPUT output;
    output.vPosition = float3(p[i].vPosition.x, p[i].vPosition.y, p[i].vPosition.z);
    output.weight = p[i].vPosition.w;
    return output;
}

// This is called once per output control point.  This sample just takes
// the control points from the vertex shader and pass them directly
// off to the domain shader.
//
// However, the "patch constant function" called "BezierConstantHS"
// is called once per PATCH, and it's the important one.
// It computes the <s,t> split positions and the "tessellation factors"

[domain("quad")]
[partitioning("integer")]
[outputtopology("triangle_cw")]
[outputcontrolpoints(OUTPUT_PATCH_SIZE)]
[patchconstantfunc("BezierConstantHS")]
HS_OUTPUT BezierHS(InputPatch<VS_CONTROL_POINT_OUTPUT, INPUT_PATCH_SIZE> p,
    uint i : SV_OutputControlPointID,
    uint PatchID : SV_PrimitiveID)
{
    return fromInputPatch(p, i);
}


// In addition to the 16 control points, the Hull Shader
// produces one of these objects for each patch.
//
// The 6 numbers called "Edges[4]" and "Inside[2]" are the
// "tessellation factors" that tell the tessellation stage
// how finely to chop the <u,v> unit square. 
//
// The "Divisions" objects called "sDivisions" and "tDivisions"
// are ignored by the tessellation stage, they will be used
// by the Domain Shader to "map" the <u,v> coordinates produced
// by the tessellation stage to the <s,t> values at which the
// Bezier or NURBS patch should be evaluated.

struct HS_CONSTANT_DATA_OUTPUT
{
    float Edges[4]  : SV_TessFactor;
    float Inside[2] : SV_InsideTessFactor;

    Divisions sDivisions : DUMMY;
    Divisions tDivisions : DUMMY2;
};

// This extracts one control point from the "InputPatch" structure,
// but then it TRANSFORMS it into "CAMERA SPACE", the space in which
// the FLATNESS MEASURE works correctly. 
float3 fromInputPatchEye(InputPatch<VS_CONTROL_POINT_OUTPUT, INPUT_PATCH_SIZE> p, int i) {
    HS_OUTPUT output = fromInputPatch(p, i);
    float4 vw = mul(float4(output.vPosition, 1), g_mView);
    return float3(vw.x, vw.y, vw.z);
}

// This takes two numbers -- a starting index and an offset -- 
// which are used to select four (4) of the control points,
// transformed into "CAMERA SPACE" and packaged into a "Curve" object.
Curve curveFromFourPoints(InputPatch<VS_CONTROL_POINT_OUTPUT, INPUT_PATCH_SIZE> p,
                          int p1, int step) {

    Curve c;
    c.p1 = fromInputPatchEye(p, p1);
    c.p2 = fromInputPatchEye(p, p1 + step);
    c.p3 = fromInputPatchEye(p, p1 + step * 2);
    c.p4 = fromInputPatchEye(p, p1 + step * 3);
    return c;
}

// This constructs a Bezier Curve using four (4) of the control points
// by calling "curveFromFourPoints" above, but then it calls "addSubdivisions".
// This recursively subdivides the curve until the flatness measure is
// exceeded, ADDING the resulting split locations to the "Divisions" object.
Divisions addDivisions(Divisions d,
                       InputPatch<VS_CONTROL_POINT_OUTPUT, INPUT_PATCH_SIZE> p,
                       int p1, int step) {

    Curve c = curveFromFourPoints(p, p1, step);
    return addSubdivisions(c, d);
}

// This is the MAIN ENTRY POINT for the Hull Shader's "patch constant function".
// It produces the "HS_CONSTANT_DATA_OUTPUT" object which contains the
// specific <s,t> values at which the patch should be split:

HS_CONSTANT_DATA_OUTPUT BezierConstantHS( InputPatch<VS_CONTROL_POINT_OUTPUT, INPUT_PATCH_SIZE> p,
                                          uint PatchID : SV_PrimitiveID )
{    
    // The patch is defined by 16 control points arranged in a 4x4 grid.
    // Each row or column of this grid consists of 4 control points
    // which may be considered the control points of a Bezier Curve.

    // The top row of 4 control points form the "t=0" boundary curve
    // The bottom row of 4 control points form the "t=1" boundary curve
    // The middle two rows, of 4 control points each, define two internal curves
    //
    // Any of these curves can be "processed", (by which we mean subdividing the curve 
    // until a flatness threshold is reached), to collect the set of "s" values at which
    // the patch is to be divided.  The -299 patent (columns 9 and 10) explains that ALL
    // four of these curves should be examined.
    //
    // Here, we arbitrarily select only ONE boundary curve to process (for s),
    // so if the two boundary curves (for s) happen to differ greatly
    // in how curved they are, and we happen to pick the less curved one,
    // then we know we might not subdivide "s" finely enough.

    Divisions sDivisions = emptyDivisions();
    //sDivisions = addDivisions(sDivisions, p, 0,  1);
    //sDivisions = addDivisions(sDivisions, p, 4,  1);
    //sDivisions = addDivisions(sDivisions, p, 8,  1);
    sDivisions = addDivisions(sDivisions, p, 12, 1);

    // We do this same thing for "t".  So, in total,
    // exactly TWO boundary curves are considered:

    Divisions tDivisions = emptyDivisions();
    //tDivisions = addDivisions(tDivisions, p, 0, 4);
    //tDivisions = addDivisions(tDivisions, p, 1, 4);
    //tDivisions = addDivisions(tDivisions, p, 2, 4);
    tDivisions = addDivisions(tDivisions, p, 3, 4);

    // These two objects, "sDivisions" and "tDivisions",
    // are emitted by this hull shader (executed once for a patch)
    // and consumed by the domain shader (which is executed for
    // each triangle vertex in the tesselated result).

    int numTIntervals = getNumSetBits(tDivisions);
    int numSIntervals = getNumSetBits(sDivisions);

    HS_CONSTANT_DATA_OUTPUT Output;
    Output.Edges[0]   = Output.Edges[2] = numTIntervals;
    Output.Edges[1]   = Output.Edges[3] = numSIntervals;
    Output.Inside[0]  = numSIntervals;
    Output.Inside[1]  = numTIntervals;
    Output.tDivisions = tDivisions;
    Output.sDivisions = sDivisions;
    return Output;
}


// ###################################################
// DOMAIN SHADER
// ###################################################

// For each vertex in the tesselated patch, the "Domain Shader" is called
// and provided with a <u,v> coordinate pair produced by the tessellation
// stage.   The "Domain Shader" has to map the <u,v> coordinates (which are
// placed evenly across the unit square) to the <s,t> coordinates (at which
// the "Hull Shader" decided the patch should be split), and then we have
// to evaluate the Bezier or NURBS patch in 3D space to locate the vertex.

// This is the "Domain Shader" output:
//
struct DS_OUTPUT
{
    // The final position after VIEW and PROJECTION transforms:
    float4 vPosition : SV_POSITION;

    // These two fields are used only for lighting,
    // which we do in the pixel shader:
    float3 vWorldPos : WORLDPOS;
    float3 vNormal   : NORMAL;
};


//--------------------------------------------------
// Evaluating a Bezier or NURBS patch
//--------------------------------------------------

float4 BernsteinBasis(float t)
{
    float invT = 1.0f - t;
    return float4(invT * invT * invT,
        3.0f * t * invT * invT,
        3.0f * t * t * invT,
        t * t * t);
}

float4 dBernsteinBasis(float t)
{
    float invT = 1.0f - t;
    return float4(-3 * invT * invT,
        3 * invT * invT - 6 * t * invT,
        6 * t * invT - 3 * t * t,
        3 * t * t);
}

// This is the regular "non-rational" Bezier Patch evaluation function.
// It returns the location in 3D space of a given <s,t> coordinate pair.
float3 EvaluateBezier (const OutputPatch<HS_OUTPUT, OUTPUT_PATCH_SIZE> bezpatch,
                       float4 BasisS,
                       float4 BasisT )
{
    float3 Value = 
        BasisT.x * ( bezpatch[0].vPosition * BasisS.x 
                   + bezpatch[1].vPosition * BasisS.y 
                   + bezpatch[2].vPosition * BasisS.z 
                   + bezpatch[3].vPosition * BasisS.w )

      + BasisT.y * ( bezpatch[4].vPosition * BasisS.x 
                   + bezpatch[5].vPosition * BasisS.y 
                   + bezpatch[6].vPosition * BasisS.z 
                   + bezpatch[7].vPosition * BasisS.w )

      + BasisT.z * ( bezpatch[8].vPosition * BasisS.x
                   + bezpatch[9].vPosition * BasisS.y 
                   + bezpatch[10].vPosition * BasisS.z 
                   + bezpatch[11].vPosition * BasisS.w )

      + BasisT.w * ( bezpatch[12].vPosition * BasisS.x
                   + bezpatch[13].vPosition * BasisS.y 
                   + bezpatch[14].vPosition * BasisS.z 
                   + bezpatch[15].vPosition * BasisS.w );

    return Value;
}

// This  is the NURBS "rational" Patch evaluation function.
// It returns the location in 3D space of a given <s,t> coordinate pair.
float3 EvaluateNURBS (const OutputPatch<HS_OUTPUT, OUTPUT_PATCH_SIZE> bezpatch,
                      float4 BasisS,
                      float4 BasisT)
{
    float3 Numerator = 
                 BasisT.x * (bezpatch[0].vPosition * bezpatch[0].weight * BasisS.x
                           + bezpatch[1].vPosition * bezpatch[1].weight * BasisS.y
                           + bezpatch[2].vPosition * bezpatch[2].weight * BasisS.z
                           + bezpatch[3].vPosition * bezpatch[3].weight * BasisS.w)

               + BasisT.y * (bezpatch[4].vPosition * bezpatch[4].weight * BasisS.x
                           + bezpatch[5].vPosition * bezpatch[5].weight * BasisS.y
                           + bezpatch[6].vPosition * bezpatch[6].weight * BasisS.z
                           + bezpatch[7].vPosition * bezpatch[7].weight * BasisS.w)

               + BasisT.z * (bezpatch[8].vPosition * bezpatch[8].weight * BasisS.x
                           + bezpatch[9].vPosition * bezpatch[9].weight * BasisS.y
                           + bezpatch[10].vPosition * bezpatch[10].weight * BasisS.z
                           + bezpatch[11].vPosition * bezpatch[11].weight * BasisS.w)

               + BasisT.w * (bezpatch[12].vPosition * bezpatch[12].weight * BasisS.x
                           + bezpatch[13].vPosition * bezpatch[13].weight * BasisS.y
                           + bezpatch[14].vPosition * bezpatch[14].weight * BasisS.z
                           + bezpatch[15].vPosition * bezpatch[15].weight * BasisS.w);

    float Denominator = 
                BasisT.x * (bezpatch[0].weight * BasisS.x
                          + bezpatch[1].weight * BasisS.y
                          + bezpatch[2].weight * BasisS.z
                          + bezpatch[3].weight * BasisS.w)

               +BasisT.y * (bezpatch[4].weight * BasisS.x
                          + bezpatch[5].weight * BasisS.y
                          + bezpatch[6].weight * BasisS.z
                          + bezpatch[7].weight * BasisS.w)

               +BasisT.z * (bezpatch[8].weight * BasisS.x
                          + bezpatch[9].weight * BasisS.y
                          + bezpatch[10].weight * BasisS.z
                          + bezpatch[11].weight * BasisS.w)

               +BasisT.w * (bezpatch[12].weight * BasisS.x
                          + bezpatch[13].weight * BasisS.y
                          + bezpatch[14].weight * BasisS.z
                          + bezpatch[15].weight * BasisS.w);
                          
    return Numerator / Denominator;
}

// This is the "Domain Shader" entry point:
// It receives a <u,v> coordinate pair from the "tessellation stage"
// along with the control points and the "HS_CONSTANT_DATA_OUTPUT" object
// produced by the "Hull Shader".
//
// The output from the domain shader will be a vertex that will go to the video card's
// rasterization pipeline and get drawn to the screen.

[domain("quad")]
DS_OUTPUT BezierDS( HS_CONSTANT_DATA_OUTPUT input, 
                    float2 UV : SV_DomainLocation,
                    const OutputPatch<HS_OUTPUT, OUTPUT_PATCH_SIZE> bezpatch )
{
    float u = UV.x;
    float v = UV.y;

    // --------------------------------------------------------
    // Shift <U,V> according to the patterns encoded in "input"
    // --------------------------------------------------------

    float s = reshift(input.sDivisions, input.Inside[0], u);
    float t = reshift(input.tDivisions, input.Inside[1], v);

    // --------------------------------------------------------
    // Evaluate the Bezier PATCH or the NURBS PATCH at the <s,t> coordinate
    // --------------------------------------------------------

    float4 BasisS = BernsteinBasis( s );
    float4 BasisT = BernsteinBasis( t );

    DS_OUTPUT Output;
    bool nurbs = (g_bNurbs > 0);
    if (!nurbs) {
        float3 WorldPos = EvaluateBezier(bezpatch, BasisS, BasisT);
        Output.vPosition = mul(float4(WorldPos, 1), g_mView);
        Output.vPosition = mul(Output.vPosition, g_mProj);
        Output.vWorldPos = WorldPos;

        // Normal vector for Bezier surface:
        float4 dBasisS = dBernsteinBasis(s);
        float4 dBasisT = dBernsteinBasis(t);
        float3 Tangent = EvaluateBezier(bezpatch, dBasisS, BasisT);
        float3 BiTangent = EvaluateBezier(bezpatch, BasisS, dBasisT);
        float3 Norm = normalize(cross(Tangent, BiTangent));

        Output.vNormal = Norm;
    } else {
        float3 WorldPos = EvaluateNURBS(bezpatch, BasisS, BasisT);
        Output.vPosition = mul(float4(WorldPos, 1), g_mView);
        Output.vPosition = mul(Output.vPosition, g_mProj);
        Output.vWorldPos = WorldPos;

        // TODO:  Note we are NOT computing the normal correctly
        // for NURBS surfaces because I'm pretty sure it involves
        // a different formula than the Bezier case.   I could do the
        // math to compute the normal correctly but haven't done it yet..

        Output.vNormal = float3(0,0,1);
    }
    return Output;
}


// ###################################################
// PIXEL SHADER
// ###################################################

// This pixel shader performs very simple N dot L lighting.
//
// It computes the angle between the normal and the line to the camera.
// In SHOULD be the angle between the normal and the line to the LIGHT.
// Hence, this has the effect of PRETENDING that there's a single light 
// at the camera's location:

float4 BezierPS( DS_OUTPUT Input ) : SV_TARGET
{
    float3 N = normalize(Input.vNormal);
    float3 L = normalize(Input.vWorldPos - g_vCameraPosWorld);


    return abs(dot(N, L)) * float4(1, 0, 0, 1);
}

// This is the pixel shader used when the program is in "wireframe" mode.
// It does NOT do any "lighting" -- it just returns the color GREEN:

float4 SolidColorPS( DS_OUTPUT Input ) : SV_TARGET
{
    // Return a solid green color
    return float4( 0, 1, 0, 1 );
}


// ##############################################################
// ##############################################################

// These remaining functions here are NOT USED for shading
// the patch, they are used only for the "extra geometry",
// namely the checkerboard floor, and the "regular triangles"
// making visible the control points and lines between them:

struct VS_INPUT {
    float3 vWorldPos : POSITION;
    float3 vNormal   : NORMAL;
    float3 vColor    : COLOR;
};

struct VS_OUTPUT
{
    float4 vPosition        : SV_POSITION;
    float4 vColor           : COLOR;
};

VS_OUTPUT ExtraGeometryVS( VS_INPUT input )
{
    VS_OUTPUT output = (VS_OUTPUT)0;
    output.vPosition = mul(float4(input.vWorldPos, 1), g_mView);
    output.vPosition = mul(output.vPosition, g_mProj);

    float3 N = input.vNormal;
    float3 L = normalize(input.vWorldPos - g_vCameraPosWorld);
    output.vColor = abs(dot(N, L)) * float4(input.vColor, 1);

    return output;
}

float4 ExtraGeometryPS (VS_OUTPUT input) : SV_TARGET
{
    return input.vColor;
}

