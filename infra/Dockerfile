FROM python:3.12-alpine

RUN pip install uv==0.5.7

COPY .python-version pyproject.toml uv.lock avionics/

WORKDIR avionics

# gcc needed for psutil only
RUN apk add gcc python3-dev musl-dev linux-headers

RUN uv sync

COPY . .

CMD uv run maxerva1.py