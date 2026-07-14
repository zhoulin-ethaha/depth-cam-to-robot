# mcp_server

FastMCP server exposing the pipeline as tools (registered in `../.mcp.json`).
It is a thin client over the RUNNING app — start `run.bat` first; the app owns
the camera/robot, tools talk to it via HTTP/WS on port 5005 (`DEPTH_APP_URL` to
override). Tools: app_status, capture_image, generate_path, load_surface,
set_surface_pose, save_toolpath, validate_toolpath. No run() tool by design —
executing robot motion stays a human action in the browser.
