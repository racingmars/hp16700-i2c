/*****************************************************************************
 * I2C serial decoder for HP 16700 series logic analyzers.
 * Matthew R. Wilson <mwilson@mattwilson.org>
 *
 * See the LICENSE file accompying this source file for copyright and
 * redistribution information. Copyright (c) 2013 by Matthew R. Wilson.
 *
 * For use with the Tool Development Kit.
 *****************************************************************************/


/**
 * TODO:
 *   - check for user cancel
 *   - custom SCL/SDA label names
 *   - make an icon
 *   - color/highlight rows for certain conditions like start?
 *   - any special handling for restart conditions?
 */


enum conditions {
    IDLE,
    READ_ADDR,
    READ_RW,
    READ_DATA,
    READ_ACK,
    INVALID
};

struct decoder {
    enum conditions     state;
    uint8_t             scl;
    uint8_t             sda;
    uint8_t             pos;
    uint8_t             byteBuffer;
};

typedef struct decoder decoder;

decoder* newDecoder();
void destroyDecoder(decoder *d);
void handleState(decoder *d, unsigned int scl, unsigned int sda,
                 long long time, TDKDataSet &dataDs, TDKDataSet &eventDs,
                 TDKLabelEntry &i2cLE, TDKLabelEntry &i2cEventsLE,
                 TDKBaseIO &io);

void execute(TDKDataGroup &dg, TDKBaseIO &io) {
    decoder *d;

    TDKDataSet ds;
    TDKDataSet dataDs;
    TDKDataSet eventDs;

    TDKLabelEntry sclLE;
    TDKLabelEntry sdaLE;
    unsigned int sclValue, sdaValue;

    TDKLabelEntry i2cLE;
    TDKLabelEntry i2cEventsLE;

    long long correlationTime;
    unsigned int origNumSamples;
    int triggerRow;
    long long time, lastTime;

    int err = 0;

    d = newDecoder();
    if(d==0) {
        io.print("Unable to allocate decoder memory.");
        return;
    }

    err = ds.attach(dg);
    if(err) {
        io.printError(err);
        destroyDecoder(d);
        return;
    }

    correlationTime = ds.getCorrelationTime();
    ds.setTimeBias();

    err = sclLE.attach(ds, "SCL");
    if(err) {
        io.printError(err);
        destroyDecoder(d);
        return;
    }

    err = sdaLE.attach(ds, "SDA");
    if(err) {
        io.printError(err);
        destroyDecoder(d);
        return;
    }

    origNumSamples = ds.getNumberOfSamples();

    ds.peekNext(time);
    triggerRow = -1;

    while(time <= nanoSec(0)) {
        ds.next(time);
        triggerRow++;
    }
    triggerRow=0;

    ds.reset();

    err = dataDs.createTimeTags(dg, "I2CData", origNumSamples,
                                triggerRow, correlationTime, nanoSec(4.0));
    if(err) {
        io.printError(err);
        destroyDecoder(d);
        return;
    }
    dataDs.setTimeBias();
    dataDs.reset();
    dataDs.displayStateNumberLabel(false);

    err = eventDs.createTimeTags(dg, "I2CEvents", origNumSamples,
                                 triggerRow, correlationTime, nanoSec(4.0));
    if(err) {
        io.printError(err);
        destroyDecoder(d);
        return;
    }
    eventDs.setTimeBias();
    eventDs.reset();
    eventDs.displayStateNumberLabel(false);

    err = i2cLE.createIntegralData(dataDs, "I2C_DATA", 8);
    if(err) {
        io.printError(err);
        destroyDecoder(d);
        return;
    }

    err = i2cEventsLE.createTextData(eventDs, "I2C_EVT", 16);
    if(err) {
        io.printError(err);
        destroyDecoder(d);
        return;
    }

    while( ds.next(time) && sclLE.next(sclValue) && sdaLE.next(sdaValue) ) {
        handleState(d, sclValue, sdaValue, time, dataDs, eventDs, i2cLE, i2cEventsLE, io);
        lastTime = time;
    }

    // try to clean up rest of new data sets. Make sure time always goes
    // forward, and filter data points we didn't end up using.
    long long eventPosition;
    long long dataPosition;

    eventDs.setStateBias();
    dataDs.setStateBias();

    eventPosition = eventDs.getPosition();
    dataPosition = dataDs.getPosition();

    eventDs.setTimeBias();
    dataDs.setTimeBias();

    while(eventDs.replaceNext(++lastTime));
    while(dataDs.replaceNext(++lastTime));

    eventDs.filterAllStates();
    dataDs.filterAllStates();

    long long f;
    for(f=0; f<eventPosition; f++) {
        eventDs.unfilter(f);
    }

    for(f=0; f<dataPosition; f++) {
        dataDs.unfilter(f);
    }

    destroyDecoder(d);

    dg.setTimeCrossCorrelation();

}

decoder* newDecoder() {
    decoder *d;
    d = (decoder*)malloc(sizeof(decoder));

    // Assume we're starting with an idle bus
    d->state = IDLE;
    d->scl = 1;
    d->sda = 1;

    return d;
}

void destroyDecoder(decoder *d) {
    free(d);
}

void handleState(decoder *d, unsigned int scl, unsigned int sda,
                 long long time, TDKDataSet &dataDs, TDKDataSet &eventDs,
                 TDKLabelEntry &i2cLE, TDKLabelEntry &i2cEventsLE,
                 TDKBaseIO &io) {
    if( d->scl == scl && d->sda == sda ) {
        // nothing changed; move on to next state on the wire
        return;
    }

    else if( d->scl == 1 && scl == 0 ) {
        // Clock going low; ignore
    }

    else if( scl == 0 ) {
        // Data changing while clock is low; ignore
    }

    // Look for start condition. This can occur at any time, even if bus isn't
    // idle.
    else if( d->scl==1 && d->sda==1 && scl==1 && sda==0 ) {
        if( d->state == IDLE ) {
            eventDs.replaceNext(time);
            i2cEventsLE.replaceNext((String)"START");
        } else {
            eventDs.replaceNext(time);
            i2cEventsLE.replaceNext((String)"START(odd)");
        }
        d->state = READ_ADDR;
        d->pos = 0;
        d->byteBuffer = 0;
    }

    // Look for stop condition. This can occur at any time.
    else if( d->scl==1 && d->sda==0 && scl==1 && sda==1) {
        eventDs.replaceNext(time);
        i2cEventsLE.replaceNext((String)"STOP");
        d->state = IDLE;
    }

    // After a start we should read the address
    else if( d->state==READ_ADDR && scl==1 ) {
        d->byteBuffer |= sda << (6-d->pos);
        d->pos++;
        if( d->pos == 7 ) {
            d->state = READ_RW;
            eventDs.replaceNext(time);
            i2cEventsLE.replaceNext((String)"ADDRESS");
            dataDs.replaceNext(time);
            i2cLE.replaceNext((unsigned int)(d->byteBuffer));
            d->pos = 0;
            d->byteBuffer = 0;
        }
    }

    // Get the R/W bit
    else if( d->state==READ_RW && scl==1 ) {
        eventDs.replaceNext(time);
        if( sda == 0 ) {
            i2cEventsLE.replaceNext((String)"WRITE");
        } else {
            i2cEventsLE.replaceNext((String)"READ");
        }
        d->state = READ_ACK;
    }

    // Get the ACK
    else if( d->state==READ_ACK && scl==1 ) {
        eventDs.replaceNext(time);
        if( sda == 0 ) {
            i2cEventsLE.replaceNext((String)"ACK");
        } else {
            i2cEventsLE.replaceNext((String)"NACK");
        }
        d->state = READ_DATA;
        d->pos = 0;
        d->byteBuffer = 0;
    }

    // Read a byte of data
    else if( d->state==READ_DATA && scl==1 ) {
        d->byteBuffer |= sda << (7-d->pos);
        d->pos++;
        if( d->pos == 8 ) {
            dataDs.replaceNext(time);
            i2cLE.replaceNext((unsigned int)(d->byteBuffer));
            eventDs.replaceNext(time);
            i2cEventsLE.replaceNext((String)"DATA");
            d->pos = 0;
            d->byteBuffer = 0;
            d->state = READ_ACK;
        }
    }

    else {
        io.print("There's a state here we haven't implemented yet");
        io.printf("  state=%d, lastSCL=%d, lastSDA=%d, scl=%d, sda=%d",
                  d->state, d->scl, d->sda, d->scl, d->sda);
    }

    d->scl = scl;
    d->sda = sda;
}

StringList getLabelNames() {
    StringList labels;
    return labels;
}

StringList getDefaultArgs() {
    StringList labels;
    return labels;
}


