"""
STL to Wavefront (.stl to .obj)
usage: blender -b -P stl_to_obj.py -- in.stl out.obj
"""
import sys
import bpy # Blender Python API

def setup():
    """ Setup the scene """
    # Delete the default objects
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

def usage():
    sys.stderr.write(__doc__)

def convert(fstl, fobj='out.obj'):
    bpy.ops.import_mesh.stl(filepath=fstl)
    bpy.ops.export_scene.obj(filepath=fobj, axis_forward='Y', axis_up='Z')

def main(argv=[]):
    args = []
    if '--' in argv:
        args = argv[argv.index('--')+1:]

    if len(args) < 2:
        usage()
        return 1

    fstl = args[0]
    fobj = args[1]
    setup()
    convert(fstl, fobj)

if __name__ == '__main__':
    main(sys.argv)
