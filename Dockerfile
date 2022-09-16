FROM skhatiri/px4

#Setting up the current tool 
COPY ./requirements.txt /src/aerialist/requirements.txt
WORKDIR /src/aerialist/
RUN pip3 install -r requirements.txt
COPY . .
RUN chmod +x ./aerialist/__main__.py
COPY ./template.env ./.env
RUN mkdir -p /io/ ./results/logs/

# ENTRYPOINT [ "./run.py" ]
# CMD [ "--help" ]




