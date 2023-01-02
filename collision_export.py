import bpy


class ExportOperator(bpy.types.Operator):
    bl_idname = "export_scene.fo2_track_cbd2_gen"
    bl_label = "Export FlatOut 2 track_cdb2.gen"

    # gets set by the file select window - internal Blender Magic or whatever.
    filepath: bpy.props.StringProperty(
        name="File Path", description="File path used for exporting the track_cdb2.gen file", maxlen=1024, default="")

    def execute(self, context):
        self.report(
            {'ERROR'}, f'export of {self.properties.filepath} not yet implemented')
        return {'FINISHED'}

    def invoke(self, context, event):
        # sets self.properties.filename and runs self.execute()
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


def export_menu_func(self, context):
    self.layout.operator(ExportOperator.bl_idname,
                         text="FlatOut 2 track_cdb2.gen")


def register():
    bpy.utils.register_class(ExportOperator)
    bpy.types.TOPBAR_MT_file_export.append(export_menu_func)


def unregister():
    bpy.types.TOPBAR_MT_file_export.remove(export_menu_func)
    bpy.utils.unregister_class(ExportOperator)
