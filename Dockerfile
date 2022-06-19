FROM skhatiri/px4

#Setting up the current tool 
COPY ./requirements.txt /src/aerialist/requirements.txt
WORKDIR /src/aerialist/
RUN pip3 install -r requirements.txt
COPY . /src/aerialist/
RUN chmod +x /src/aerialist/run.py
RUN chmod +x /src/aerialist/entry.sh
COPY ./template.env /src/aerialist/.env
RUN mkdir -p /io/ /src/aerialist/results/logs/

# ENTRYPOINT [ "./run.py" ]
# CMD [ "--help" ]




