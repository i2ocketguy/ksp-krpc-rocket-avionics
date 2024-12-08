# KSP KRPC Rocket Avionics

## Running

This project uses `uv` for dependency management. To install the dependencies run `uv sync`. 
Dependencies will be stored in the .venv folder within this project. You can point your IDE to this venv for local runtime.
Alternatively you can run scripts using the `uv` command in a way which access the venv, e.g. `uv run maxerva1.py`

## Craft files

Check out https://kerbalx.com/I2ocketGuy/craft, especially:

- [Maxerva-1 Kerbalx](https://kerbalx.com/I2ocketGuy/Maxerva-I-Kerbalx)
- [Minerva-1 FR](https://kerbalx.com/I2ocketGuy/Minerva-I-FR)
- [Maxerva-I CEV FR](https://kerbalx.com/I2ocketGuy/Maxerva-I-CEV-FR)

## Live telem view

WIP docker compose based orchestration available for launch with

`make launch_metrics` will launch the system using host networking and requires the following ports available:
- 3000
- 9090
- 8012
- 50000
- 50001

access the system via browser at localhost:3000

```mermaid

flowchart TD
    
  human -- localhost:3000  --> grafana -- localhost:9090 --> prometheus -- localhost:8012/healthz --> avionics
  
  grafana -- telem graphs --> human
  avionics -- telem --> prometheus
  prometheus -- telem --> grafana
```