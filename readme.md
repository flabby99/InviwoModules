# Work in progress Layered Depth Images in Inviwo

## TODO
- Control the point size based on distances, or if this does not work well, convert my vertex shader code to fragment shader code and keep the fixed sizes.
- Check if there is an advantage to setting the position of opacity 0 points to out of clip space.
- If not using point size, set up clipping renderprocessor with a fragment shader because then I can discard fragments with 0 opacity.
- Review all of my code TODOs.
- Don't splat the central view once confident on the correctness.
- Split planes non-linearly (make non-linear distances).