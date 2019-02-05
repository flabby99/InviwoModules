# Work in progress Layered Depth Images in Inviwo

## TODO
- Try ray based splitting instead of coming up with designated clip planes
- Control the point size based on distances, or if this does not work well, convert my vertex shader code to fragment shader code and keep the fixed sizes.
- Check if there is an advantage to setting the position of opacity 0 points to out of clip space.
- If not using point size, set up clipping renderprocessor with a fragment shader because then I can discard fragments with 0 opacity.
- Fix the problem with the plane clipping distances (see heart dataset, long side).
- Review all of my code TODOs.
- Don't keep initialising new images, only do it on resize.
- Remove printing of plane distances once I am confident about their correctness.
