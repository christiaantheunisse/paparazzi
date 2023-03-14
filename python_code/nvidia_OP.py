import numpy as np
from scipy.ndimage import center_of_mass

def nvidia_OP(images: np.ndarray, history: int = 6, **kwargs) -> np.ndarray:
    """
    Calculates the optical flow for a set images.
    
    Args:
        images: a numpy array of grayscale images; (B x H x W)
        history: the no. of past images to use to calculate the flow
        kwargs: these are passed to the `center_of_mass` function
        
    Returns:
        flows: a 2D numpy array for every image with the 2D flow vector; (B x h_steps x w_steps x 2)
    """
    assert history % 2 == 0, "The `history` should be an even number!"
    assert len(images.shape) == 3, "The images shoud be of the size B x H x W: so a batch of grayscale images!"
    
    print("The first few frames are empty, because it uses past values to calculate the optical flow...\n")
    print("You can set the `box_size: tuple[int, int]` and the `step_size: tuple[int, int]` for the box "\
          "that is moved over the image wherein the center of mass is calculated. See `nvidia_OP.py` for more info.")
    CoMs = CoM_over_img(images, **kwargs)
    
    diffs = []
    for i in range(len(CoMs)):
        mean_prev = CoMs[i - history : i - history // 2].mean(axis=0)
        mean = CoMs[i - history // 2 : i].mean(axis=0)
        diff = mean - mean_prev
        diff = np.abs(diff.clip(-5, 5))
        diffs.append(diff)
      
    return np.array(diffs)


def CoM_over_img(images: np.ndarray, box_size: tuple[int, int] = (20, 50), step_size: tuple[int, int] = (20, 20)) -> np.ndarray:
    """
    Calculates the center of mass for a set of images.
    
    Args:
        images: a numpy array of grayscale images; (B x H x W)
        box_size: a tuple with the size of the boxes of which to calculate the center of mass; (height, width)
        step_size: a tuple with the size of the steps with which to move the boxes; (step in height-direction, width direction)
    Returns:
        CoMs: the center of masses for different parts of the images. (B x h_steps x w_steps)
    """
    box_size_i, box_size_j = box_size
    i_max, j_max = images.shape[1:3]
    i_step_size, j_step_size = step_size
    i_steps, j_steps = (i_max - box_size_i) // i_step_size, (j_max - box_size_j) // j_step_size, 

    CoMs = []
    for image in images:
        CoM = np.zeros((i_steps, j_steps, 2))
        for i in range(i_steps):
            for j in range(j_steps):
                image_region = image[
                    i * i_step_size : i * i_step_size + box_size_i,
                    j * j_step_size : j * j_step_size + box_size_j,
                ]
                if image_region.sum() == 0:
                    CoM[i, j] = [box_size_i // 2, box_size_j // 2]
                else:
                    CoM[i, j] = center_of_mass(image_region)
                
        CoMs.append(CoM)

    return np.array(CoMs)

def OP_to_depth(flows: np.ndarray, flow_direction: str = 'x', W: float = 0.2, clip_flow: float = 5, to_img: bool = False) -> np.ndarray:
    """
    Converts the depth from the optical flow. It averages the optical flow over the height-direction,
    so an 1D array is returned with the depth along the width-direction.
    
    Args:
        flows: the optical for a batch of images, where the flow is a 2D vector; (B x H x W x 2)
        flow-direction: the dimension of the flow to select.
                        {'x' = x-direction, 'y' = y-direction, 'xy' = sqrt(x**2 + y**2)}
        W: represents the forward speed of the drone in the equation. However, depending on the OP algorithm
           it might lose its physical meaning and have a random value. So try to tune it.
        clip_flow: value to clip the OP, because it might have some unstable noise. 
        to_img: to output the depth in image format to easy visualisation
        
    Returns:
        dist: 1D array with the depth along the width-direction of the image.
    """
    
    if flow_direction == 'x':
        u = flows[...,1]  # So the OP in x-direction is selected
    elif flow_direction == 'y':
        u = flows[...,0]
    elif flow_direction == 'xy':
        u = np.linalg.norm(flows, axis=-1)
    else:
        assert False, f"{flow_direction} is not a valid option for the flow_direction."\
        "The options are: \{'x' = x-direction, 'y' = y-direction, 'xy' = sqrt(x**2 + y**2)\} " 
    
    distances = []
    for flow in u:
        flow = flow.clip(-clip_flow, clip_flow)
        mean = flow.mean(axis=0)  # mean in vertical direction
        # The formula is: Z / W = x / u
        x_range = np.linspace(-len(mean)/2, len(mean)/2, len(mean))  # create a vector with x-values
        min_center_val = 5
        # The x-values in the center are very close the zero and give therefore unstable Z values; so make them a bit larger
        x_range[((x_range < min_center_val) & (x_range >= 0))] = min_center_val
        x_range[(x_range > -min_center_val) & (x_range < 0)] = -min_center_val
        # Z = x / u * W
        dist = x_range/mean * W
        dist[dist < 0] = 10  # distances smaller than 0 are discarded
        dist = dist.clip(0, 10) # distances further away than 10 m are set to 10 m
        # Create a image from the depth to make it easy to visualise
        if to_img:
            dist_img = np.repeat(np.expand_dims(dist, axis=0), 20, axis=0)
        else:
            dist_img = dist
        distances.append(dist_img)
        
    
    return np.array(distances)

def plot_widget(values: np.ndarray) -> None:
    """
    Create a jupyter notebook widget to visualise for example the depth in a plot.
    Args:
        values: for example the depth; (B x W)
        
    """
    @widgets.interact(index=(0, len(values)-1))
    def f(index=0):
        plt.plot(values[index])
        plt.xlabel("array index")
        plt.ylabel("value")
        plt.ylim(0, 11)