# Camera Controls

The LaneZero viewer now supports interactive camera controls for better navigation and visualization.

## Mouse Controls

### Zoom
- **Mouse Wheel Up**: Zoom in
- **Mouse Wheel Down**: Zoom out

### Pan (Move Camera)
- **Left Mouse Button + Drag**: Pan the view in any direction
- The cursor changes to a closed hand while panning

### Rotate
- **Right Mouse Button + Drag**: Rotate the view
- Horizontal mouse movement controls rotation angle
- The cursor changes to a cross while rotating

## Keyboard Controls

### Pan (Arrow Keys)
- **Left Arrow**: Move view left
- **Right Arrow**: Move view right
- **Up Arrow**: Move view up
- **Down Arrow**: Move view down

### Zoom
- **Plus (+) or Equal (=)**: Zoom in
- **Minus (-)**: Zoom out

### Rotate
- **R**: Rotate view by 15 degrees clockwise
- **0**: Reset rotation to 0 degrees
- **Shift + R**: Reset camera to default position (zoom, pan, and rotation)

## Camera Limits

- **Minimum Zoom**: 0.1x
- **Maximum Zoom**: 100x
- **Rotation Range**: 0-360 degrees (continuous)
- **Pan**: Unlimited

## Tips

1. Click on the viewer window to ensure it has keyboard focus before using keyboard controls
2. Use mouse wheel for quick zoom adjustments
3. Use Shift+R to quickly reset the camera if you get lost
4. Combine pan and zoom to focus on specific areas of interest
5. Use rotation to view the scene from different angles

## Technical Details

The camera controls are implemented in `src/viewer/RenderWidget.hpp` and `src/viewer/RenderWidget.cpp`. The following member variables control the camera state:

- `m_scale`: Current zoom level (default: 10.0)
- `m_offset_x`, `m_offset_y`: Camera position offset (default: 100.0, 400.0)
- `m_rotation`: Current rotation angle in degrees (default: 0.0)
- `m_zoom_factor`: Zoom speed multiplier (1.1)

The camera transformation is applied in the `paintEvent` method using Qt's transformation matrix.
