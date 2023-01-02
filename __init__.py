from . import collision_import, collision_export

bl_info = {
    "name": "track_cdb2 format",
    "blender": (3, 4, 0),
    "category": "Import-Export",
    "description": "Support for FlatOut 2's track_cdb2.gen track collision format",
}


def register():
    collision_import.register()
    collision_export.register()


def unregister():
    collision_import.unregister()
    collision_export.unregister()
