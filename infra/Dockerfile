FROM python:3.12

COPY .python-version pyproject.toml uv.lock avionics/

WORKDIR avionics

RUN pip install uv && uv sync

COPY . .

CMD uv run maxerva1.py