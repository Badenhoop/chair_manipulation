import vtk
from mayavi_array_handler import vtk2array

def vtk_reader(objfile):
    obj_reader = vtk.vtkOBJReader()
    obj_reader.SetFileName(objfile)
    obj_reader.Update()
    return obj_reader.GetOutput()


def vtk_get_points(polydata):
    return vtk2array(polydata.GetPoints().GetData())


def vtk_get_center_of_mass(polydata):
    com_filter = vtk.vtkCenterOfMass()
    com_filter.SetUseScalarsAsWeights(False)
    com_filter.SetInputData(polydata)
    com_filter.Update()
    return np.array(com_filter.GetCenter())


def filepath_add_suffix(filepath, suffix, sep="-"):
    fileprefix, ext = osp.splitext(filepath)
    return sep.join((fileprefix, suffix)) + ext


def vtk_writer(polydata, objfile):
    assert polydata
    from packaging import version
    assert (version.parse(vtk.vtkVersion().GetVTKVersion()) >=
            version.parse("8.2.0")), "OBJWriter is not available before 8.2.0"
    obj_writer = vtk.vtkOBJWriter()
    obj_writer.SetFileName(objfile)
    obj_writer.SetInputData(polydata)
    obj_writer.Update()


def vtk_shift_polydata(polydata, shift):
    translation = vtk.vtkTransform()
    translation.Translate(*shift)
    transform_filter = vtk.vtkTransformPolyDataFilter()
    transform_filter.SetTransform(translation)
    transform_filter.SetInputData(polydata)
    transform_filter.Update()
    return transform_filter.GetOutput()


def vtk_get_volume(polydata):
    massprop = vtk.vtkMassProperties()
    massprop.SetInputData(polydata)
    massprop.Update()
    return massprop.GetVolume()


def write_corresponding_mtl_file(objfile, centered_objfile):
    objbaseprefix, _ = osp.splitext(osp.basename(objfile))
    centered_objfileprefix, _ = osp.splitext(centered_objfile)

    try:
        os.symlink(objbaseprefix + ".mtl", centered_objfileprefix + ".mtl")
    except FileExistsError:
        pass


def mesh_mass_properties(objfile, density=10):
    polydata = vtk_reader(objfile)
    points = vtk_get_points(polydata)
    mins = np.quantile(points, 0.05, axis=0)#  points.min(axis=0)
    maxs = np.quantile(points, 0.95, axis=0)# points.max(axis=0)
    dims = (maxs - mins)

    volume = vtk_get_volume(polydata)

    center = vtk_get_center_of_mass(polydata)

    inertia = trimesh_inertia_mat(objfile)

    centered_objfile = filepath_add_suffix(objfile, "centered")
    unit = unitstr2rescale_factor(parse_sketchup_file_units(objfile))
    rescale_factor = np.ones(3) * unit
    rescaled_dims = dims * rescale_factor
    return dict(dims=rescaled_dims.tolist(),
                scale=rescale_factor.tolist(),
                mass=density * volume * unit**3 ,
                center=(center * unit).tolist(),
                inertia=(inertia * unit**2).tolist())

def mesh_mass_properties_old(objfile,
                         desired_max_dim=dict(
                             bed=3,
                             bookcase=3,
                             chair=1,
                             desk=1.5,
                             table=1.5,
                             sofa=2.0,
                             wardrobe=3)):
    _, ikea_type, *ikea_subtype = ikea_names[0].split("_")
    maxdim = desired_max_dim[ikea_type]
    polydata = vtk_reader(objfile)
    points = vtk_get_points(polydata)
    mins = np.quantile(points, 0.05, axis=0)#  points.min(axis=0)
    maxs = np.quantile(points, 0.95, axis=0)# points.max(axis=0)

    center = vtk_get_center_of_mass(polydata)

    volume = vtk_get_volume(polydata)

    centered_objfile = filepath_add_suffix(objfile, "centered")
    ### Does not work because vtkOBJReader does not read textures
    ### pywavefront does but it does not write objects
    ### Mapping pywavefront textures to a vtkActor + PolyData will be hard.
    ### vtkOBJImporter and vtkOBJExporter read obj files for rendering
    ### but do not export to polydata where vertex manipulation can be done.
    # shifted_polydata = vtk_shift_polydata(polydata, -center)
    # vtk_writer(shifted_polydata, centered_objfile)
    # write_corresponding_mtl_file(objfile, centered_objfile)
    dims = (maxs - mins)
    mesh_maxdim = dims.max()
    rescale_factor = np.ones(3) * maxdim / mesh_maxdim
    rescaled_dims = dims * rescale_factor
    return rescaled_dims.tolist(), rescale_factor.tolist(), volume, center.tolist()

