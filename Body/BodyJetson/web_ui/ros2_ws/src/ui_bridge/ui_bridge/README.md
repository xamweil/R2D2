## Start / rebuild (only this container)
```bash
docker compose up -d --build web_ui
````

## Open in browser

* `http://198.168.66.2:8000/`

## API endpoints

* Health: `GET /health`
* WebSocket robot state stream: `ws://198.168.66.2:8000/ws`
* Call allowlisted service: `POST /call/allowed`

## Dev mode (dangerous)

Enable generic service calls / subscriptions:

* set `DEV_UNSAFE=1` in `docker-compose.yml` for `web_ui` and restart the container.

## Notes

* Uses `network_mode: host` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (must match other ROS containers).
* Frontend is served from `web_ui/frontend` mounted to `/home/ros/frontend`.

```
