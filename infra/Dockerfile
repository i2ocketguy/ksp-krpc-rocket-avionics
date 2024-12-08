FROM python:3.12

COPY . avionics

WORKDIR avionics

RUN pip install uv && uv sync

CMD uv run maxerva1.py