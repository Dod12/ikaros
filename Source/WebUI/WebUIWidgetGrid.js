class WebUIWidgetGrid extends WebUIWidgetGraph
{
    static template()
    {
        return [
            {'name': "DATA", 'control':'header'},
            
            {'name':'title', 'default':"", 'type':'string', 'control': 'textedit'},
            {'name':'module', 'default':"", 'type':'source', 'control': 'textedit'},
            {'name':'source', 'default':"", 'type':'source', 'control': 'textedit'},
            {'name':'red', 'default':"", 'type':'source', 'control': 'textedit'},
            {'name':'green', 'default':"", 'type':'source', 'control': 'textedit'},
            {'name':'blue', 'default':"", 'type':'source', 'control': 'textedit'},
            {'name':'min', 'default':0, 'type':'float', 'control': 'textedit'},
            {'name':'max', 'default':1, 'type':'float', 'control': 'textedit'},
            {'name':'labels', 'default':"", 'type':'string', 'control': 'textedit'},
            {'name':'labelWidth', 'default':100, 'type':'int', 'control': 'textedit'},

            {'name': "CONTROL", 'control':'header'},
            
            {'name':'command', 'default':"", 'type':'source', 'control': 'textedit'},
            {'name':'parameter', 'default':"", 'type':'source', 'control': 'textedit'},
            {'name':'valueHigh', 'default':1, 'type':'float', 'control': 'textedit'},
            {'name':'valueLow', 'default':0, 'type':'float', 'control': 'textedit'},
            
            {'name': "STYLE", 'control':'header'},

            {'name':'color', 'default':'', 'type':'string', 'control': 'textedit'},
            {'name':'fill', 'default':"gray", 'type':'string', 'control': 'menu', 'values': "gray,fire,spectrum,custom,rgb"},
            {'name':'colorTable', 'default':'', 'type':'string', 'control': 'textedit'},
            {'name':'lineWidth', 'default':1, 'type':'float', 'control': 'textedit'},
            {'name':'shape', 'default':"rectangle", 'type':'string', 'control': 'menu', 'values': "rectangle,square,circle"},
            {'name':'size', 'default':1, 'type':'float', 'control': 'textedit'},

            {'name': "COORDINATE SYSTEM", 'control':'header'},

            {'name':'scales', 'default':"no", 'type':'string', 'control': 'menu', 'values': "yes,no,invisible", 'class':'true'},
            {'name':'min_x', 'default':0, 'type':'float', 'control': 'textedit'},
            {'name':'max_x', 'default':1, 'type':'float', 'control': 'textedit'},
            {'name':'min_y', 'default':0, 'type':'float', 'control': 'textedit'},
            {'name':'max_y', 'default':1, 'type':'float', 'control': 'textedit'},
            {'name':'flipXAxis', 'default':"no", 'type':'string', 'control': 'menu', 'values': "yes,no"},
            {'name':'flipYAxis', 'default':"no", 'type':'string', 'control': 'menu', 'values': "yes,no"},
            {'name':'flipXCanvas', 'default':"no", 'type':'string', 'control': 'menu', 'values': "yes,no"},
            {'name':'flipYCanvas', 'default':"no", 'type':'string', 'control': 'menu', 'values': "yes,no"},

            {'name': "FRAME", 'control':'header'},

            {'name':'show_title', 'default':true, 'type':'bool', 'control': 'checkbox'},
            {'name':'show_frame', 'default':true, 'type':'bool', 'control': 'checkbox'},
            {'name':'style', 'default':"", 'type':'string', 'control': 'textedit'},
            {'name':'frame-style', 'default':"", 'type':'string', 'control': 'textedit'}
        ]
    }

    init()
    {
        super.init();
        this.data = [];

        this.onclick = function (evt)
        {
            if(!this.data)
                return;
            
            if(!this.parameters.command && !this.parameters.parameter)
            {
                let s = "";
                for(let r of this.data)
                {
                    for(let c of r)
                        s += c+"\t";
                    s += "\n"
                }
                alert(s);
                return;
            }
            
            let lw = this.parameters.labels ? parseInt(this.parameters.labelWidth) : 0;
            let r = this.canvasElement.getBoundingClientRect();
            let x = Math.floor(this.data[0].length*(evt.clientX - r.left - this.format.spaceLeft - lw)/(r.width - this.format.spaceLeft - this.format.spaceRight- lw));
            let y = Math.floor(this.data.length*(evt.clientY - r.top - this.format.spaceTop)/(r.height - this.format.spaceTop - this.format.spaceBottom));

            if(x < 0 || x >= this.data[0].length || y < 0 || y >= this.data.length)
                return;

            if(this.parameters.command && this.parameters.module)
                this.get("/command/"+this.parameters.module+"/"+this.parameters.command+"/"+x+"/"+y+"/"+this.parameters.valueHigh);
            
            else if(this.parameters.parameter && this.parameters.module)
            {
                if(this.data[y][x] < this.parameters.valueHigh)
                    this.get("/control/"+this.parameters.module+"/"+this.parameters.parameter+"/"+x+"/"+y+"/"+this.parameters.valueHigh);
                else
                    this.get("/control/"+this.parameters.module+"/"+this.parameters.parameter+"/"+x+"/"+y+"/"+this.parameters.valueLow);
            }
        }
    }

    requestData(data_set)
    {
        if(this.parameters.fill == "rgb")
        {
            data_set.add(this.parameters['module']+"."+this.parameters['red']);
            data_set.add(this.parameters['module']+"."+this.parameters['green']);
            data_set.add(this.parameters['module']+"."+this.parameters['blue']);
        }
        else
            data_set.add(this.parameters['module']+"."+this.parameters['source']);
    }

    drawPlotHorizontal(width, height, index, transform)
    {
        let d = this.data;
        let rows = 0;
        let cols = 0;
        
        if(this.parameters.fill == "rgb")
        {
            rows = d[0].length;
            cols = d[0][0].length;
        }
        else
        {
            rows = d.length;
            cols = d[0].length;
        }
        
        this.canvas.lineWidth = this.format.lineWidth;
        this.canvas.textAlign = 'left';
        this.canvas.textBaseline = 'middle';

        let ct = LUT_gray;
        if(this.parameters.fill == 'fire')
            ct = LUT_fire;
        else if(this.parameters.fill == 'spectrum')
            ct = LUT_spectrum;

        if(this.parameters.colorTable != "")
        {
            let q = this.parameters.colorTable;
            ct = this.parameters.colorTable.split(',');
        }

        let labels = this.parameters.labels === "" ? [] : this.parameters.labels.split(',');
        let ln = labels.length;
        let ls = (ln ? parseInt(this.parameters.labelWidth) : 0);
        let n = ct.length;
        let dx = (width-ls)/cols;
        let dy = height/rows;
        let sx = dx*this.parameters.size;
        let sy = dy*this.parameters.size;

        if(this.parameters.shape == 'square' || this.parameters.shape == 'circle')
        {
            let minimum = Math.min(sx, sy);
            sx = minimum;
            sy = minimum;
        }

        if(this.parameters.fill == "rgb")
        {
            for(var i=0; i<rows; i++)
                {
                    if(ln)
                    {
                        this.canvas.fillStyle = "black";    // FIXME: Should really use the default color form the stylesheet
                        this.canvas.fillText(labels[i % (ln+1)].trim(), 0, dy*i+dy/2);
                    }

                    for(var j=0; j<cols; j++)
                    {
                        this.setColor(i+j);
                        this.canvas.beginPath();

                        let r = (255*d[0][i][j]).toString(16);
                        let g = (255*d[1][i][j]).toString(16);
                        let b = (255*d[2][i][j]).toString(16);
                        
                        this.canvas.fillStyle = '#'+r+g+b;
                        
                        if(this.parameters.shape == 'circle')
                            this.canvas.arc(ls+dx*j+dx/2, dy*i+dy/2, sx/2, 0, 2*Math.PI);
                        else
                            this.canvas.rect(ls+dx*j+dx/2-sx/2, dy*i+dy/2-sy/2, sx, sy);

                        this.canvas.fill();
                        this.canvas.stroke();
                    }
                }
        }
        else
        {
            for(var i=0; i<rows; i++)
            {
                if(ln)
                {
                    this.canvas.fillStyle = "black";    // FIXME: Should really use the default color form the stylesheet
                    this.canvas.fillText(labels[i % (ln+1)].trim(), 0, dy*i+dy/2);
                }

                for(var j=0; j<cols; j++)
                {
                    this.setColor(i+j);
                    this.canvas.beginPath();
                    let f = (d[i][j]-this.parameters.min)/(this.parameters.max-this.parameters.min);
                    let ix = Math.min(Math.floor(n*f), n-1);
                    this.canvas.fillStyle = ct[ix].trim();
                    
                    if(this.parameters.shape == 'circle')
                        this.canvas.arc(ls+dx*j+dx/2, dy*i+dy/2, sx/2, 0, 2*Math.PI);
                    else
                        this.canvas.rect(ls+dx*j+dx/2-sx/2, dy*i+dy/2-sy/2, sx, sy);

                    this.canvas.fill();
                    this.canvas.stroke();
                }
            }
        }
    }

    update(d) // default for Graph - should not be needed here
    {
        if(!d)
            return;
        
        try {
            if(this.parameters.fill == "rgb")
            {
                let m = this.parameters['module'];
                let r = this.parameters['red'];
                let g = this.parameters['green'];
                let b = this.parameters['blue'];
                this.data = [d[m][r], d[m][g], d[m][b]];
            }
            
            else
            {
                let m = this.parameters['module'];
                let s = this.parameters['source'];
                this.data = d[m][s];
            }

            if(!this.data)
                return;

            this.canvas.setTransform(1, 0, 0, 1, -0.5, -0.5);
            this.canvas.clearRect(0, 0, this.width, this.height);
            this.canvas.translate(this.format.marginLeft, this.format.marginTop); //

            this.drawHorizontal(1, 1);  // Draw grid over image - should be Graph:draw() with no arguments
        }
        catch(err)
        {
        }
    }
};


webui_widgets.add('webui-widget-grid', WebUIWidgetGrid);
