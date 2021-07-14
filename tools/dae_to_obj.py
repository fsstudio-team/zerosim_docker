"""
Collada to Wavefront (.dae to .obj)
usage: blender -b -P dae_obj.py -- in.dae out.obj
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

def convert(fdae, fobj='out.obj'):
    bpy.ops.wm.collada_import(filepath=fdae)
    bpy.ops.export_scene.obj(filepath=fobj, axis_forward='Y', axis_up='Z')

def main(argv=[]):
    args = []
    if '--' in argv:
        args = argv[argv.index('--')+1:]

    if len(args) < 2:
        usage()
        return 1

    fdae = args[0]
    fobj = args[1]
    setup()
    convert(fdae, fobj)

if __name__ == '__main__':
    main(sys.argv)