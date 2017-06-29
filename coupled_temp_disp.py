from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *

# Parameters in meters.
A = 0.6;
B = 0.72;
D = 5.0;
H = 3.0;

I = 0.01;
N = 5.0;
M = 10.0;

B1 = 0.05 + I;
D1 = 0.2 * N;
H1 = (0.05) * (M);

ROOM_TEMPERATURE = 300;

HEAT_FLUX = 200;
HEAT_FLUX_ANGLE = M;

FILM_COEFF_EXTERIOR = 2;
FILM_COEFF_INTERIOR = 1;

CONDUCTIVITY = 15;

# Part Leg
mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=20.0)
mdb.models['Model-1'].sketches['__profile__'].Line(point1=(-3.0, 0.0), point2=(
    3.0, 0.0))
mdb.models['Model-1'].sketches['__profile__'].HorizontalConstraint(
    addUndoState=False, entity=
    mdb.models['Model-1'].sketches['__profile__'].geometry[2])
mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.0, 3.0), point2=(
    0.0, -3.0))
mdb.models['Model-1'].sketches['__profile__'].VerticalConstraint(addUndoState=
    False, entity=mdb.models['Model-1'].sketches['__profile__'].geometry[3])
mdb.models['Model-1'].sketches['__profile__'].setAsConstruction(objectList=(
    mdb.models['Model-1'].sketches['__profile__'].geometry[2], ))
mdb.models['Model-1'].sketches['__profile__'].setAsConstruction(objectList=(
    mdb.models['Model-1'].sketches['__profile__'].geometry[3], ))
mdb.models['Model-1'].sketches['__profile__'].FixedConstraint(entity=
    mdb.models['Model-1'].sketches['__profile__'].geometry[3])
mdb.models['Model-1'].sketches['__profile__'].FixedConstraint(entity=
    mdb.models['Model-1'].sketches['__profile__'].geometry[2])
mdb.models['Model-1'].sketches['__profile__'].Line(point1=(-1.0, 0.0), point2=(
    -1.0, 1.5))
mdb.models['Model-1'].sketches['__profile__'].VerticalConstraint(addUndoState=
    False, entity=mdb.models['Model-1'].sketches['__profile__'].geometry[4])
mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
    addUndoState=False, entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[2], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[4])
mdb.models['Model-1'].sketches['__profile__'].CoincidentConstraint(
    addUndoState=False, entity1=
    mdb.models['Model-1'].sketches['__profile__'].vertices[4], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[2])
mdb.models['Model-1'].sketches['__profile__'].Line(point1=(-1.0, 1.5), point2=(
    0.0, 1.5))
mdb.models['Model-1'].sketches['__profile__'].HorizontalConstraint(
    addUndoState=False, entity=
    mdb.models['Model-1'].sketches['__profile__'].geometry[5])
mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
    addUndoState=False, entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[4], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[5])
mdb.models['Model-1'].sketches['__profile__'].CoincidentConstraint(
    addUndoState=False, entity1=
    mdb.models['Model-1'].sketches['__profile__'].vertices[6], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[3])
mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.0, 1.5), point2=(
    0.0, 0.998853206634521))
mdb.models['Model-1'].sketches['__profile__'].VerticalConstraint(addUndoState=
    False, entity=mdb.models['Model-1'].sketches['__profile__'].geometry[6])
mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
    addUndoState=False, entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[5], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[6])
mdb.models['Model-1'].sketches['__profile__'].CoincidentConstraint(
    addUndoState=False, entity1=
    mdb.models['Model-1'].sketches['__profile__'].vertices[7], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[3])
mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.0, 
    0.998853206634521), point2=(-0.5, 0.998853206634521))
mdb.models['Model-1'].sketches['__profile__'].HorizontalConstraint(
    addUndoState=False, entity=
    mdb.models['Model-1'].sketches['__profile__'].geometry[7])
mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
    addUndoState=False, entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[6], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[7])
mdb.models['Model-1'].sketches['__profile__'].Line(point1=(-0.5, 
    0.998853206634521), point2=(-0.5, 0.0))
mdb.models['Model-1'].sketches['__profile__'].VerticalConstraint(addUndoState=
    False, entity=mdb.models['Model-1'].sketches['__profile__'].geometry[8])
mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
    addUndoState=False, entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[7], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[8])
mdb.models['Model-1'].sketches['__profile__'].CoincidentConstraint(
    addUndoState=False, entity1=
    mdb.models['Model-1'].sketches['__profile__'].vertices[9], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[2])
mdb.models['Model-1'].sketches['__profile__'].DistanceDimension(entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[7], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[5], textPoint=(
    0.638718068599701, 1.40977942943573), value=B1)
mdb.models['Model-1'].sketches['__profile__'].DistanceDimension(entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[5], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[2], textPoint=(
    -1.94261336326599, 0.815290987491608), value=(B/2.0) )
mdb.models['Model-1'].sketches['__profile__'].DistanceDimension(entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[8], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[4], textPoint=(
    -0.78934919834137, 0.694885432720184), value=B1)
mdb.models['Model-1'].sketches['__profile__'].DistanceDimension(entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[4], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[3], textPoint=(
    -0.143399864435196, 0.919063270092011), value=A)
mdb.models['Model-1'].sketches['__profile__'].copyMirror(mirrorLine=
    mdb.models['Model-1'].sketches['__profile__'].geometry[2], objectList=(
    mdb.models['Model-1'].sketches['__profile__'].geometry[6], 
    mdb.models['Model-1'].sketches['__profile__'].geometry[5], 
    mdb.models['Model-1'].sketches['__profile__'].geometry[7], 
    mdb.models['Model-1'].sketches['__profile__'].geometry[4], 
    mdb.models['Model-1'].sketches['__profile__'].geometry[8]))
mdb.models['Model-1'].Part(dimensionality=THREE_D, name='Part-Leg', type=
    DEFORMABLE_BODY)
mdb.models['Model-1'].parts['Part-Leg'].BaseSolidExtrude(depth=(H-A), sketch=
    mdb.models['Model-1'].sketches['__profile__'])
del mdb.models['Model-1'].sketches['__profile__']

# Part Top
mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=20.0)
mdb.models['Model-1'].sketches['__profile__'].Line(point1=(-3.0, 0.0), point2=(
    3.0, 0.0))
mdb.models['Model-1'].sketches['__profile__'].HorizontalConstraint(
    addUndoState=False, entity=
    mdb.models['Model-1'].sketches['__profile__'].geometry[2])
mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.0, 3.0), point2=(
    0.0, -3.0))
mdb.models['Model-1'].sketches['__profile__'].VerticalConstraint(addUndoState=
    False, entity=mdb.models['Model-1'].sketches['__profile__'].geometry[3])
mdb.models['Model-1'].sketches['__profile__'].setAsConstruction(objectList=(
    mdb.models['Model-1'].sketches['__profile__'].geometry[2], ))
mdb.models['Model-1'].sketches['__profile__'].setAsConstruction(objectList=(
    mdb.models['Model-1'].sketches['__profile__'].geometry[3], ))
mdb.models['Model-1'].sketches['__profile__'].FixedConstraint(entity=
    mdb.models['Model-1'].sketches['__profile__'].geometry[3])
mdb.models['Model-1'].sketches['__profile__'].FixedConstraint(entity=
    mdb.models['Model-1'].sketches['__profile__'].geometry[2])
mdb.models['Model-1'].sketches['__profile__'].Line(point1=(-1.0, 0.0), point2=(
    -1.0, 1.5))
mdb.models['Model-1'].sketches['__profile__'].VerticalConstraint(addUndoState=
    False, entity=mdb.models['Model-1'].sketches['__profile__'].geometry[4])
mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
    addUndoState=False, entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[2], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[4])
mdb.models['Model-1'].sketches['__profile__'].CoincidentConstraint(
    addUndoState=False, entity1=
    mdb.models['Model-1'].sketches['__profile__'].vertices[4], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[2])
mdb.models['Model-1'].sketches['__profile__'].Line(point1=(-1.0, 1.5), point2=(
    0.0, 1.5))
mdb.models['Model-1'].sketches['__profile__'].HorizontalConstraint(
    addUndoState=False, entity=
    mdb.models['Model-1'].sketches['__profile__'].geometry[5])
mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
    addUndoState=False, entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[4], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[5])
mdb.models['Model-1'].sketches['__profile__'].CoincidentConstraint(
    addUndoState=False, entity1=
    mdb.models['Model-1'].sketches['__profile__'].vertices[6], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[3])
mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.0, 1.5), point2=(
    0.0, 0.998853206634521))
mdb.models['Model-1'].sketches['__profile__'].VerticalConstraint(addUndoState=
    False, entity=mdb.models['Model-1'].sketches['__profile__'].geometry[6])
mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
    addUndoState=False, entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[5], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[6])
mdb.models['Model-1'].sketches['__profile__'].CoincidentConstraint(
    addUndoState=False, entity1=
    mdb.models['Model-1'].sketches['__profile__'].vertices[7], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[3])
mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.0, 
    0.998853206634521), point2=(-0.5, 0.998853206634521))
mdb.models['Model-1'].sketches['__profile__'].HorizontalConstraint(
    addUndoState=False, entity=
    mdb.models['Model-1'].sketches['__profile__'].geometry[7])
mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
    addUndoState=False, entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[6], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[7])
mdb.models['Model-1'].sketches['__profile__'].Line(point1=(-0.5, 
    0.998853206634521), point2=(-0.5, 0.0))
mdb.models['Model-1'].sketches['__profile__'].VerticalConstraint(addUndoState=
    False, entity=mdb.models['Model-1'].sketches['__profile__'].geometry[8])
mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
    addUndoState=False, entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[7], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[8])
mdb.models['Model-1'].sketches['__profile__'].CoincidentConstraint(
    addUndoState=False, entity1=
    mdb.models['Model-1'].sketches['__profile__'].vertices[9], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[2])
mdb.models['Model-1'].sketches['__profile__'].DistanceDimension(entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[7], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[5], textPoint=(
    0.638718068599701, 1.40977942943573), value=B1)
mdb.models['Model-1'].sketches['__profile__'].DistanceDimension(entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[5], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[2], textPoint=(
    -1.94261336326599, 0.815290987491608), value=(B/2.0) )
mdb.models['Model-1'].sketches['__profile__'].DistanceDimension(entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[8], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[4], textPoint=(
    -0.78934919834137, 0.694885432720184), value=B1)
mdb.models['Model-1'].sketches['__profile__'].DistanceDimension(entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[4], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[3], textPoint=(
    -0.143399864435196, 0.919063270092011), value=A)
mdb.models['Model-1'].sketches['__profile__'].copyMirror(mirrorLine=
    mdb.models['Model-1'].sketches['__profile__'].geometry[2], objectList=(
    mdb.models['Model-1'].sketches['__profile__'].geometry[6], 
    mdb.models['Model-1'].sketches['__profile__'].geometry[5], 
    mdb.models['Model-1'].sketches['__profile__'].geometry[7], 
    mdb.models['Model-1'].sketches['__profile__'].geometry[4], 
    mdb.models['Model-1'].sketches['__profile__'].geometry[8]))
mdb.models['Model-1'].Part(dimensionality=THREE_D, name='Part-Top', type=
    DEFORMABLE_BODY)
mdb.models['Model-1'].parts['Part-Top'].BaseSolidExtrude(depth=(D), sketch=
    mdb.models['Model-1'].sketches['__profile__'])
del mdb.models['Model-1'].sketches['__profile__']

# Part-Top Front Wall
mdb.models['Model-1'].ConstrainedSketch(gridSpacing=0.12, name='__profile__', 
    sheetSize=5.15, transform=
    mdb.models['Model-1'].parts['Part-Top'].MakeSketchTransform(
    sketchPlane=mdb.models['Model-1'].parts['Part-Top'].faces[8], 
    sketchPlaneSide=SIDE1, 
    sketchUpEdge=mdb.models['Model-1'].parts['Part-Top'].edges[6], 
    sketchOrientation=RIGHT, 
	origin=((-A)+B1, (-B/2)+B1, (H-A))))

mdb.models['Model-1'].parts['Part-Top'].projectReferencesOntoSketch(filter=
    COPLANAR_EDGES, sketch=mdb.models['Model-1'].sketches['__profile__'])
mdb.models['Model-1'].sketches['__profile__'].rectangle(
		point1=(0.0, 0.0), 
		point2=((B - 2*B1), (-A+B1)))
mdb.models['Model-1'].parts['Part-Top'].SolidExtrude(depth=B1, 
    flipExtrudeDirection=ON, sketch=
    mdb.models['Model-1'].sketches['__profile__'], sketchOrientation=RIGHT, 
    sketchPlane=mdb.models['Model-1'].parts['Part-Top'].faces[8], 
    sketchPlaneSide=SIDE1, sketchUpEdge=
    mdb.models['Model-1'].parts['Part-Top'].edges[6])
del mdb.models['Model-1'].sketches['__profile__']

# Part-Top Back Wall
mdb.models['Model-1'].ConstrainedSketch(gridSpacing=0.12, name='__profile__', 
    sheetSize=5.15, transform=
    mdb.models['Model-1'].parts['Part-Top'].MakeSketchTransform(
    sketchPlane=mdb.models['Model-1'].parts['Part-Top'].faces[9], 
    sketchPlaneSide=SIDE1, 
    sketchUpEdge=mdb.models['Model-1'].parts['Part-Top'].edges[28], 
    sketchOrientation=RIGHT, origin=((-A)+B1, (B/2)-B1, 0.0)))
mdb.models['Model-1'].parts['Part-Top'].projectReferencesOntoSketch(filter=
    COPLANAR_EDGES, sketch=mdb.models['Model-1'].sketches['__profile__'])
mdb.models['Model-1'].sketches['__profile__'].rectangle(
		point1=(0.0, 0.0), 
		point2=((B - 2*B1), (-A+B1)))
mdb.models['Model-1'].parts['Part-Top'].SolidExtrude(depth=B1, 
    flipExtrudeDirection=ON, sketch=
    mdb.models['Model-1'].sketches['__profile__'], sketchOrientation=RIGHT, 
    sketchPlane=mdb.models['Model-1'].parts['Part-Top'].faces[9], 
    sketchPlaneSide=SIDE1, sketchUpEdge=
    mdb.models['Model-1'].parts['Part-Top'].edges[28])
del mdb.models['Model-1'].sketches['__profile__']

# Part-Arm
mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=20.0)
mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(0.0, 0.0), 
    point2=(-1.0, 1.0))
mdb.models['Model-1'].sketches['__profile__'].DistanceDimension(entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[4], entity2=
    mdb.models['Model-1'].sketches['__profile__'].geometry[2], textPoint=(
    -0.052617073059082, 1.76755881309509), value=2*B1)
mdb.models['Model-1'].sketches['__profile__'].DistanceDimension(entity1=
    mdb.models['Model-1'].sketches['__profile__'].geometry[3], entity2=
    mdb.models['Model-1'].sketches['__profile__'].vertices[3], textPoint=(
    0.875574827194214, 0.229361891746521), value=B1)
mdb.models['Model-1'].Part(dimensionality=THREE_D, name='Part-Arm', type=
    DEFORMABLE_BODY)
mdb.models['Model-1'].parts['Part-Arm'].BaseSolidExtrude(depth=(A+H1), sketch=
    mdb.models['Model-1'].sketches['__profile__'])
del mdb.models['Model-1'].sketches['__profile__']

# Assembly
# Assembly Right Leg
mdb.models['Model-1'].rootAssembly.DatumCsysByDefault(CARTESIAN)
mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='Part-Top-1', 
    part=mdb.models['Model-1'].parts['Part-Top'])
mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='Part-Leg-1', 
    part=mdb.models['Model-1'].parts['Part-Leg'])
mdb.models['Model-1'].rootAssembly.rotate(angle=90.0, axisDirection=(0.0, 
    -0.18, 0.0), axisPoint=(-0.6, 0.36, 5.0), instanceList=('Part-Leg-1', ))
mdb.models['Model-1'].rootAssembly.FaceToFace(clearance=0.0, fixedPlane=
    mdb.models['Model-1'].rootAssembly.instances['Part-Top-1'].faces[10], flip=
    OFF, movablePlane=
    mdb.models['Model-1'].rootAssembly.instances['Part-Leg-1'].faces[0])
mdb.models['Model-1'].rootAssembly.FaceToFace(clearance=0.0, fixedPlane=
    mdb.models['Model-1'].rootAssembly.instances['Part-Top-1'].faces[7], flip=
    ON, movablePlane=
    mdb.models['Model-1'].rootAssembly.instances['Part-Leg-1'].faces[8])

# Assembly Left Leg
mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='Part-Leg-2', 
    part=mdb.models['Model-1'].parts['Part-Leg'])
mdb.models['Model-1'].rootAssembly.rotate(angle=-90.0, axisDirection=(0.0, 
    -0.43, 0.0), axisPoint=(-0.6, 0.86, 5.0), instanceList=('Part-Leg-2', ))
mdb.models['Model-1'].rootAssembly.FaceToFace(clearance=0.0, fixedPlane=
    mdb.models['Model-1'].rootAssembly.instances['Part-Top-1'].faces[9], flip=
    OFF, movablePlane=
    mdb.models['Model-1'].rootAssembly.instances['Part-Leg-2'].faces[0])
mdb.models['Model-1'].rootAssembly.FaceToFace(clearance=0.0, fixedPlane=
    mdb.models['Model-1'].rootAssembly.instances['Part-Top-1'].faces[7], flip=
    ON, movablePlane=
    mdb.models['Model-1'].rootAssembly.instances['Part-Leg-2'].faces[9])

# Assembly Part-Arm
mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='Part-Arm-1', 
    part=mdb.models['Model-1'].parts['Part-Arm'])
mdb.models['Model-1'].rootAssembly.rotate(angle=90.0, axisDirection=(0.0, 
    -0.18, 0.0), axisPoint=(-0.6, 0.18, 0.0), instanceList=('Part-Arm-1', ))
mdb.models['Model-1'].rootAssembly.FaceToFace(clearance=0.0, fixedPlane=
    mdb.models['Model-1'].rootAssembly.instances['Part-Top-1'].faces[5], flip=
    ON, movablePlane=
    mdb.models['Model-1'].rootAssembly.instances['Part-Arm-1'].faces[3])
mdb.models['Model-1'].rootAssembly.FaceToFace(clearance=-(A+D1), fixedPlane=
    mdb.models['Model-1'].rootAssembly.instances['Part-Top-1'].faces[9], flip=
    OFF, movablePlane=
    mdb.models['Model-1'].rootAssembly.instances['Part-Arm-1'].faces[2])
mdb.models['Model-1'].rootAssembly.FaceToFace(clearance=0.0, fixedPlane=
    mdb.models['Model-1'].rootAssembly.instances['Part-Top-1'].faces[4], flip=
    OFF, movablePlane=
    mdb.models['Model-1'].rootAssembly.instances['Part-Arm-1'].faces[4])

# Assembly Part-Table-Full Merge 
mdb.models['Model-1'].rootAssembly.InstanceFromBooleanMerge(domain=GEOMETRY, 
    instances=(mdb.models['Model-1'].rootAssembly.instances['Part-Top-1'], 
    mdb.models['Model-1'].rootAssembly.instances['Part-Leg-1'], 
    mdb.models['Model-1'].rootAssembly.instances['Part-Leg-2'], 
    mdb.models['Model-1'].rootAssembly.instances['Part-Arm-1']), name=
    'Part-Table-Full', originalInstances=SUPPRESS)
	
# Step
# Step Coupled Temp-Disp, Steady_State
# http://abaqus.software.polimi.it/v6.13/books/usb/default.htm?startat=pt03ch06s05at19.html
mdb.models['Model-1'].CoupledTempDisplacementStep(amplitude=STEP, cetol=None, 
    creepIntegration=None, deltmx=100.0, matrixStorage=SOLVER_DEFAULT, name=
    'Step-1', previous='Initial', solutionTechnique=SEPARATED)
mdb.models['Model-1'].steps['Step-1'].setValues(creepIntegration=None, nlgeom=
    OFF)
mdb.models['Model-1'].steps['Step-1'].setValues(amplitude=RAMP, cetol=None, 
    creepIntegration=None, deltmx=None, response=STEADY_STATE)
mdb.models['Model-1'].steps['Step-1'].setValues(creepIntegration=None, 
    initialInc=0.01, maxInc=0.01)

# Material
# https://www.youtube.com/watch?v=CT--YwUcFj4
mdb.models['Model-1'].Material(name='Material')
mdb.models['Model-1'].materials['Material'].Elastic(table=((210000000000.0, 
    0.3), ))
mdb.models['Model-1'].materials['Material'].Expansion(table=((1.6e-05, ), ))
mdb.models['Model-1'].materials['Material'].Conductivity(table=((CONDUCTIVITY, ), ))
# Assign Material
mdb.models['Model-1'].HomogeneousSolidSection(material='Material', name=
    'Section-Full', thickness=None)
mdb.models['Model-1'].parts['Part-Table-Full'].Set(cells=
    mdb.models['Model-1'].parts['Part-Table-Full'].cells.getSequenceFromMask((
    '[#1 ]', ), ), name='Set-1')
mdb.models['Model-1'].parts['Part-Table-Full'].SectionAssignment(offset=0.0, 
    offsetField='', offsetType=MIDDLE_SURFACE, region=
    mdb.models['Model-1'].parts['Part-Table-Full'].sets['Set-1'], sectionName=
    'Section-Full', thicknessAssignment=FROM_SECTION)
	
# Partition (For heat flux)
mdb.models['Model-1'].rootAssembly.regenerate()
mdb.models['Model-1'].parts['Part-Table-Full'].PartitionCellByPlanePointNormal(
    cells=
    mdb.models['Model-1'].parts['Part-Table-Full'].cells.getSequenceFromMask((
    '[#1 ]', ), ), normal=
    mdb.models['Model-1'].parts['Part-Table-Full'].edges[20], point=
    mdb.models['Model-1'].parts['Part-Table-Full'].vertices[15])
	
## Load
# BC: Bottom Surface Room Temperature
mdb.models['Model-1'].TemperatureBC(amplitude=UNSET, createStepName='Step-1', 
    distributionType=UNIFORM, fieldName='', fixed=OFF, magnitude=ROOM_TEMPERATURE, name=
    'BC-Room_Temperature_Bottom_Surface', region=Region(
    faces=mdb.models['Model-1'].rootAssembly.instances['Part-Table-Full-1'].faces.getSequenceFromMask(
    mask=('[#110000 ]', ), )))
# BC: Bottom Surface Displacement
mdb.models['Model-1'].DisplacementBC(amplitude=UNSET, createStepName='Step-1', 
    distributionType=UNIFORM, fieldName='', fixed=OFF, localCsys=None, name=
    'BC-Displacement_Bottom', region=Region(
    faces=mdb.models['Model-1'].rootAssembly.instances['Part-Table-Full-1'].faces.getSequenceFromMask(
    mask=('[#110000 ]', ), )), u1=0.0, u2=UNSET, u3=UNSET, ur1=0.0, ur2=0.0, 
    ur3=0.0)
# BC: Bottom Edge Displacement
mdb.models['Model-1'].DisplacementBC(amplitude=UNSET, createStepName='Step-1', 
    distributionType=UNIFORM, fieldName='', fixed=OFF, localCsys=None, name=
    'BC-Displacement_Bottom_Edges', region=Region(
    edges=mdb.models['Model-1'].rootAssembly.instances['Part-Table-Full-1'].edges.getSequenceFromMask(
    mask=('[#40880800 #72840000 #f6 ]', ), )), u1=UNSET, u2=UNSET, u3=0.0, ur1=
    0.0, ur2=0.0, ur3=0.0)
# BC: Bottom Vertex  Displacement
mdb.models['Model-1'].DisplacementBC(amplitude=UNSET, createStepName='Step-1', 
    distributionType=UNIFORM, fieldName='', fixed=OFF, localCsys=None, name=
    'BC-Displacement_Bottom_Vertex', region=Region(
    vertices=mdb.models['Model-1'].rootAssembly.instances['Part-Table-Full-1'].vertices.getSequenceFromMask(
    mask=('[#31b0c00 #1f580 ]', ), )), u1=UNSET, u2=0.0, u3=UNSET, ur1=0.0, 
    ur2=0.0, ur3=0.0)

# Load Heat Flux
Q1 = (sqrt(2.0) / 2.0) * cos(degrees(HEAT_FLUX_ANGLE)) * HEAT_FLUX;
Q2 = (sqrt(2.0) / 2.0) * cos(degrees(HEAT_FLUX_ANGLE)) * HEAT_FLUX;
Q3 = sin(degrees(HEAT_FLUX_ANGLE)) * HEAT_FLUX;
# Q1
mdb.models['Model-1'].SurfaceHeatFlux(createStepName='Step-1', magnitude=Q1, 
    name='Heat_Flux_Q1_Load', region=Region(
    side1Faces=mdb.models['Model-1'].rootAssembly.instances['Part-Table-Full-1'].faces.getSequenceFromMask(
    mask=('[#40 ]', ), )))
# Q2
mdb.models['Model-1'].SurfaceHeatFlux(createStepName='Step-1', magnitude=Q2, 
    name='Heat_Flux_Q2_Load', region=Region(
    side1Faces=mdb.models['Model-1'].rootAssembly.instances['Part-Table-Full-1'].faces.getSequenceFromMask(
    mask=('[#40000000 ]', ), )))
# Q3
mdb.models['Model-1'].SurfaceHeatFlux(createStepName='Step-1', magnitude=Q3, 
    name='Heat_Flux_Q3_Load', region=Region(
    side1Faces=mdb.models['Model-1'].rootAssembly.instances['Part-Table-Full-1'].faces.getSequenceFromMask(
    mask=('[#20 ]', ), )))

# Interaction, Heat Convection
## Exterior
mdb.models['Model-1'].FilmCondition(createStepName='Step-1', definition=
    EMBEDDED_COEFF, filmCoeff=FILM_COEFF_EXTERIOR, filmCoeffAmplitude='', name=
    'Convection_Exterior', sinkAmplitude='', sinkDistributionType=UNIFORM, 
    sinkFieldName='', sinkTemperature=ROOM_TEMPERATURE, surface=Region(
    side1Faces=mdb.models['Model-1'].rootAssembly.instances['Part-Table-Full-1'].faces.getSequenceFromMask(
    mask=('[#7a0effe8 ]', ), )))
## Interior
mdb.models['Model-1'].FilmCondition(createStepName='Step-1', definition=
    EMBEDDED_COEFF, filmCoeff=FILM_COEFF_INTERIOR, filmCoeffAmplitude='', name=
    'Convection_Interior', sinkAmplitude='', sinkDistributionType=UNIFORM, 
    sinkFieldName='', sinkTemperature=ROOM_TEMPERATURE, surface=Region(
    side1Faces=mdb.models['Model-1'].rootAssembly.instances['Part-Table-Full-1'].faces.getSequenceFromMask(
    mask=('[#5e00016 ]', ), )))

# Mesh
mdb.models['Model-1'].parts['Part-Table-Full'].setElementType(elemTypes=(
    ElemType(elemCode=DC3D8, elemLibrary=STANDARD), ElemType(elemCode=DC3D6, 
    elemLibrary=STANDARD), ElemType(elemCode=C3D4T, elemLibrary=STANDARD)), 
    regions=(
    mdb.models['Model-1'].parts['Part-Table-Full'].cells.getSequenceFromMask((
    '[#3 ]', ), ), ))
mdb.models['Model-1'].parts['Part-Table-Full'].seedPart(deviationFactor=0.1, 
    minSizeFactor=0.1, size=0.05)
mdb.models['Model-1'].parts['Part-Table-Full'].setMeshControls(elemShape=TET, 
    regions=
    mdb.models['Model-1'].parts['Part-Table-Full'].cells.getSequenceFromMask((
    '[#3 ]', ), ), technique=FREE)
mdb.models['Model-1'].parts['Part-Table-Full'].generateMesh()