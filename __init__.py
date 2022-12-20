import bpy

bl_info = {
    "name": "track_cdb2 format",
    "blender": (3, 4, 0),
    "category": "Import-Export",
    "description": "Support for FlatOut 2's track_cdb2.gen track collision format",
}

class ImportOperator(bpy.types.Operator):
    bl_idname = "import_scene.fo2_track_cbd2_gen"
    bl_label = "Import FlatOut 2 track_cdb2.gen"

    #gets set by the file select window - internal Blender Magic or whatever.
    filepath: bpy.props.StringProperty(name="File Path", description="File path used for importing the track_cdb2.gen file", maxlen= 1024, default="")

    def execute(self, context):
        self.ImportStart()
        return {'FINISHED'}

    def invoke(self, context, event):
        #sets self.properties.filename and runs self.execute()
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def ImportStart(self):
        with open(self.properties.filepath) as file:
            pass
        self.report( { 'ERROR' }, f'import of {self.properties.filepath} not yet implemented')

def import_menu_func(self, context):
    self.layout.operator(ImportOperator.bl_idname, text="FlatOut 2 track_cdb2.gen")

def register():
    bpy.utils.register_class(ImportOperator)
    bpy.types.TOPBAR_MT_file_import.append(import_menu_func)
def unregister():
    bpy.types.TOPBAR_MT_file_import.remove(import_menu_func)
    bpy.utils.unregister_class(ImportOperator)
