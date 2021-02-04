import numpy as np
from sympy.geometry import Segment
from sympy import symbols
import cv2

def draw_angled_text(object_name: str, text_loc: np.ndarray, text_angle: float, sensor_image: np.ndarray):
    """
    Draws text at an angle to the image

    Parameters
    ----------
    object_name : str
        name of the object
    text_loc : numpy.ndarray
        Desired location of the text in the image
    text_angle : float
        Angle of the text.
    sensor_image : numpy.ndarray
        Input image

    Returns
    -------
    numpy.ndarray
        Resultant image
    """
    text_color = (32,112,207)
    text_image = np.zeros(sensor_image.shape, dtype=np.uint8)
    M = cv2.getRotationMatrix2D(tuple(text_loc), text_angle, 1)
    cv2.putText(text_image,
                object_name, 
                tuple(text_loc), 
                cv2.FONT_HERSHEY_PLAIN, 
                1,
                text_color,
                2)
    text_image = cv2.warpAffine(text_image, M, (text_image.shape[1], text_image.shape[0]))
    mask = np.dstack([(text_image > 0)])
    bg = sensor_image.copy()
    np.copyto(bg, text_image, where=mask)
    return bg

def segment_arb_pts(seg, 
                    n_pts: int=10, 
                    is_rand: bool=False,
                    sub_val_range: list=[0, 1]) -> np.ndarray:
    
    """Generates arbitrary point(s) on a geometric segment 

    Parameters
    ----------
    seg : Segment | list | np.ndarray 
        A geometric segment or an array containing the coordinates of the segment. 
        Size should be 2x2
    n_pts : int, optional
        Number of arbitrary points to be generated, by default 1
    generate : bool, optional
        If `True` it will generate `n_pts` number of arbirary
        points, else will take the `sub_vals` to generate
        arbitrar points, by default False
    is_rand : bool, optional
        If `True` it will generate random values, else will generate
        evenly spaced number(s) over the interval [0,1], 
        by default False
    sub_val_range : list, optional
        A list of length 2 containing the range within which the 
        values to be substituted will be generated, by default [0, 1]

    Returns
    -------
    np.ndarray
        Contains all the generated arbitrary points on the segment.
        Shape: (n_pts, 2)

    Raises
    ------
    TypeError
        If `seg` is neither Segment, list, or np.ndarray type.
    """

    if not isinstance(seg, Segment):
        
        if isinstance(seg, np.ndarray):
            assert seg.shape == (2,2), 'Segment should have two 2D points'
            seg = Segment(*seg)
        elif isinstance(seg, list):
            seg = np.array(seg)
            assert seg.shape == (2,2), 'Segment should have two 2D points'
            seg = Segment(*seg)
        else:
            raise TypeError(f'seg is of type: {type(seg)}, but it needs to be either type Segment, np.ndarray, or list')
    

    assert n_pts >= 1, 'Number of substitute values should be more than 0'
    # Either generate random values or evenly spaced number(s)
    # over the interval [0,1]
    low, high = sub_val_range
    if is_rand:
        sub_vals = np.random.uniform(low=low, high=high, size=(n_pts))
    else:
        sub_vals = np.linspace(start=low, stop=high, num=n_pts)
                                
    t = symbols('t')
    arb_pts = []
    
    for val in sub_vals:
        arb_pts.append(list(seg.arbitrary_point(t).subs(t, val).evalf()))
    
    return np.array(arb_pts, dtype=np.float)


def shrink_bbox(bbox_cor, shrink_val: int=0.9) -> np.ndarray:
    """Shrinks the area of the bounding box by reducing the 
    diagonals' length of the boxes to some ratio.

    Parameters
    ----------
    bbox_cor : list | np.ndarray
        Coordinates of the bounding box.
    shrink_val : int, optional
        Shrinking ratio of the bbox, by default 0.75  
    
        Notes
        -----
        If the `shrink_val` is more than 1.0,
        the bbox will expand.

    Returns
    -------
    np.ndarray
        Coordinate of the shrinked bbox.

    Raises
    ------
    TypeError
        If the type of `bbox_cor` is not list or np.ndarray
    
    Warning
    -------
    This method yields in longer run time. For example,
    if detecting through `sift` and `flann` based matcher takes ~0.05s,
    this function takes ~0.75s. Use this function if you need to use it
    once.
    """
    
    if not isinstance(bbox_cor, np.ndarray):
        
        if isinstance(bbox_cor, list):
            bbox_cor = np.array(bbox_cor)
        else:
            raise TypeError(f'bbox_cor needs to be the type of either list or np.ndarray not {type(bbox_cor)}')
    
    center_cor = (bbox_cor[0] + bbox_cor[2])/2
    upd_bbox_cor = np.zeros((4,2), dtype=np.int16)
    
    for i, cor in enumerate(bbox_cor):
        # Generate 2 arbitrary points (required for the segment_arb_method) 
        # on the segment: one at the center, onother for the shrink value.
        # Take the 2nd element for update bbox coordiante. 
        upd_bbox_cor[i] = segment_arb_pts([center_cor.tolist(), cor.tolist()], 
                                          n_pts=2, sub_val_range=[0, shrink_val])[1]

    return upd_bbox_cor
    
def points_on_triangle(v: np.ndarray, n:int) -> np.ndarray:
    """Generates uniformly distributed points on a given triangle.

    Parameters
    ----------
    v : ndarray
        Coordinate of the three vertices of the triangle.
    n : int
        Number of points to be generated

    Returns
    -------
    np.ndarray
        Generated random points.
    """
    
    x = np.sort(np.random.rand(2, n), axis=0)
    return np.column_stack([x[0], x[1]-x[0], 1.0-x[1]]) @ v        