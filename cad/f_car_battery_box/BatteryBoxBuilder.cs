using System;
using System.IO;
using System.Runtime.InteropServices;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

public static class BatteryBoxBuilder
{
    static ISldWorks Connect() { return (ISldWorks)Marshal.GetActiveObject("SldWorks.Application"); }
    public static string OpenV8ForFlushRevision(string folder)
    {
        try { sw=Connect(); }
        catch(COMException) { sw=(ISldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application")); }
        sw.Visible=true;
        string source=Path.GetFullPath(Path.Combine(folder,"F_car_battery_box_V8_beams56_plate_fixed.SLDPRT"));
        int errors=0,warnings=0;
        model=sw.GetOpenDocumentByName(source) as IModelDoc2;
        if(model==null) model=(IModelDoc2)sw.OpenDoc6(source,1,1,"",ref errors,ref warnings);
        Check(model!=null,"Cannot open V8; error="+errors);
        sw.ActivateDoc3(model.GetTitle(),false,0,ref errors);
        return InspectCurrentGeometry(folder);
    }
    public static string Inspect()
    {
        ISldWorks app = Connect();
        IModelDoc2 doc = app.ActiveDoc as IModelDoc2;
        return "Version=" + app.RevisionNumber() + "\nTemplate=" + app.GetUserPreferenceStringValue((int)swUserPreferenceStringValue_e.swDefaultTemplatePart) + "\nActive=" + (doc == null ? "none" : doc.GetTitle());
    }
    public static string InspectSlotFaces()
    {
        sw=Connect(); model=sw.ActiveDoc as IModelDoc2;
        Check(model!=null,"No active part");
        string report=model.GetTitle()+" unsaved="+model.GetSaveFlag();
        foreach(IBody2 body in (object[])((IPartDoc)model).GetBodies2(0,true))
        foreach(IFace2 face in (object[])body.GetFaces())
        {
            ISurface s=(ISurface)face.GetSurface();
            if(s.IsCylinder()) report+="\nCyl="+String.Join(",",(double[])s.CylinderParams)+" box="+String.Join(",",(double[])face.GetBox());
        }
        return report;
    }
    public static string LengthenRoofFloorSlotsV11(string folder,string desktopFolder)
    {
        sw=Connect(); model=sw.ActiveDoc as IModelDoc2;
        Check(model!=null && model.GetTitle().StartsWith("F_car_battery_box_V10_width105_aligned_H"),"Expected current V10");
        string basename="F_car_battery_box_V11_eight_slots_plus2mm";
        Check(!File.Exists(Path.Combine(folder,basename+".SLDPRT")) && !File.Exists(Path.Combine(desktopFolder,basename+".3mf")),"V11 already exists");
        Check(((IPartDoc)model).FeatureByName("V11 eight roof and floor slots length plus2mm")==null,"V11 already applied");
        sketches=model.SketchManager; math=(IMathUtility)sw.GetMathUtility();
        Check(sketches.ActiveSketch==null,"Finish active sketch first");
        // Exact end arc centers and radii read from the existing custom holes, mm.
        double[,] slots={
            {-34.6201070077444,-23.6201070077444,-21,1.55866905573237},
            {-15.276494209721,-4.72350579027901,-20.8136337560698,1.5},
            {3.37273248786049,14.3727324878605,-20.8136337560698,1.54663927096493},
            {21.0299790419536,32.0299790419536,-21,1.45947104138353}
        };
        IBody2 original=(IBody2)((object[])((IPartDoc)model).GetBodies2(0,true))[0];
        // Validate both top and bottom arc faces before touching geometry.
        for(int i=0;i<4;i++) for(int end=0;end<2;end++) for(int level=0;level<2;level++)
        {
            int found=0;
            foreach(IFace2 face in (object[])original.GetFaces())
            {
                ISurface surface=(ISurface)face.GetSurface(); if(!surface.IsCylinder()) continue;
                double[] c=(double[])surface.CylinderParams, b=(double[])face.GetBox();
                if(Math.Abs(c[0]-M(slots[i,end]))<1e-8 && Math.Abs(c[1]-M(slots[i,2]))<1e-8 && Math.Abs(c[6]-M(slots[i,3]))<1e-8 && Math.Abs(b[2]-M(level==0?0:33))<1e-8 && Math.Abs(b[5]-M(level==0?2:35))<1e-8) found++;
            }
            Check(found==1,"Source slot geometry differs from inspected shape");
        }
        double before=VolumeMm3(), removedExpected=0;
        int[] preferences={(int)swUserPreferenceToggle_e.swSketchAutomaticRelations,(int)swUserPreferenceToggle_e.swSketchInferFromModel,(int)swUserPreferenceToggle_e.swSketchInference};
        bool[] previous=new bool[preferences.Length];
        for(int i=0;i<preferences.Length;i++) { previous[i]=sw.GetUserPreferenceToggle(preferences[i]); sw.SetUserPreferenceToggle(preferences[i],false); }
        try {
        FaceSketch(0,0,35,0,0,1);
        for(int i=0;i<4;i++)
        {
            double shift=i==3?1:0;
            double[] a=Local(slots[i,0]-1+shift,slots[i,2],35), b=Local(slots[i,1]+1+shift,slots[i,2],35);
            Check(sketches.CreateSketchSlot(0,0,M(2*slots[i,3]),a[0],a[1],0,b[0],b[1],0,0,0,0,1,true)!=null,"V11 slot creation failed");
            removedExpected+=2*slots[i,3]*2*4; // width * extra length * two 2mm walls
            if(i>0)
            {
                double previousShift=i-1==3?1:0;
                double gap=slots[i,0]-1+shift-slots[i,3]-(slots[i-1,1]+1+previousShift+slots[i-1,3]);
                Check(gap>=2,"Insufficient inter-slot web");
            }
        }
        EndSketch("V11 preserve slot widths add2mm rightmost pair offset1mm");
        } finally { for(int i=0;i<preferences.Length;i++) sw.SetUserPreferenceToggle(preferences[i],previous[i]); }
        Cut(35,"V11 eight roof and floor slots length plus2mm");
        Check(model.ForceRebuild3(false),"V11 rebuild failed");
        Check(Math.Abs(before-VolumeMm3()-removedExpected)<0.05,"Unexpected removed material; verify before saving");
        object[] bodies=(object[])((IPartDoc)model).GetBodies2(0,true);
        Check(bodies.Length==1,"V11 must remain one solid");
        double[] bounds=(double[])((IBody2)bodies[0]).GetBodyBox(), expected={-.055,-.056,0,.055,.019,.035};
        for(int i=0;i<6;i++) Check(Math.Abs(bounds[i]-expected[i])<1e-7,"V11 overall size changed");
        model.ClearSelection2(true); model.ViewZoomtofit2(); model.GraphicsRedraw2();
        Check(model.SaveBMP(Path.Combine(folder,basename+"_preview.bmp"),1400,900),"V11 preview failed");
        string saved=Save(Path.Combine(folder,basename+".SLDPRT"))+"\n"+Save(Path.Combine(desktopFolder,basename+".3mf"))+"\n"+Save(Path.Combine(folder,basename+".STEP"));
        return saved+"\nEight slots each length+2 mm, widths unchanged. Rightmost top/bottom pair moved+1 mm; minimum web2.651 mm.\nVolume="+VolumeMm3()+"; removed="+(before-VolumeMm3())+"; expected="+removedExpected;
    }
    public static string RetryV11WithoutInference(string folder,string desktopFolder)
    {
        sw=Connect(); model=sw.ActiveDoc as IModelDoc2;
        Check(model!=null && model.GetTitle().StartsWith("F_car_battery_box_V10_width105_aligned_H"),"Expected in-progress V11 on V10");
        IFeature own=(IFeature)((IPartDoc)model).FeatureByName("V11 eight roof and floor slots length plus2mm");
        Check(own!=null,"No own failed feature to replace");
        model.ClearSelection2(true); Check(own.Select2(false,0),"Cannot select own feature");
        Check(model.Extension.DeleteSelection2((int)swDeleteSelectionOptions_e.swDelete_Absorbed),"Cannot remove own failed feature");
        model.ForceRebuild3(false);
        Check(Math.Abs(VolumeMm3()-49002.269)<0.01,"Source volume was not restored");
        return LengthenRoofFloorSlotsV11(folder,desktopFolder);
    }
    public static string InspectCurrentGeometry(string folder)
    {
        sw=Connect(); model=sw.ActiveDoc as IModelDoc2;
        Check(model!=null,"No active model");
        string report="Active="+model.GetTitle()+"; unsaved="+model.GetSaveFlag()+"; volume="+VolumeMm3();
        object[] bodies=(object[])((IPartDoc)model).GetBodies2(0,true);
        foreach(IBody2 body in bodies) report+="\nBounds m="+String.Join(",",(double[])body.GetBodyBox());
        IFeature f=(IFeature)model.FirstFeature();
        while(f!=null) { report+="\n"+f.Name+" | "+f.GetTypeName2(); f=(IFeature)f.GetNextFeature(); }
        model.SaveBMP(Path.Combine(folder,"before_board_relocation.bmp"),1400,900);
        return report;
    }
    public static string MountBoardOnBeams(string folder,string desktopFolder)
    {
        sw=Connect(); model=sw.ActiveDoc as IModelDoc2;
        Check(model!=null && model.GetTitle().StartsWith("F_car_battery_box_V5_hollow_H_t2",StringComparison.OrdinalIgnoreCase),"Current source changed; inspect before proceeding");
        string basename="F_car_battery_box_V6_board_on_beams";
        string native=Path.Combine(folder,basename+".SLDPRT");
        string output=Path.Combine(desktopFolder,basename+".3mf");
        Check(!File.Exists(native) && !File.Exists(output),"V6 output already exists");
        Check(((IPartDoc)model).FeatureByName("V6 hollow beam extensions 15 mm")==null,"V6 already in progress");
        sketches=model.SketchManager; math=(IMathUtility)sw.GetMathUtility();
        double initialVolume=VolumeMm3();
        FaceSketch(-51.5,-65,30,0,-1,0);
        foreach(double x in new double[]{-47.5,47.5})
        {
            Rectangle(x-5,-65,25,x+5,-65,35);
            Rectangle(x-3,-65,27,x+3,-65,33);
        }
        EndSketch("V6 extend 10 x 10 hollow beams with 2 mm walls");
        Boss(15,"V6 hollow beam extensions 15 mm");
        Check(Math.Abs(VolumeMm3()-initialVolume-1920)<0.1,"Hollow beam extension volume mismatch");

        FaceSketch(-47.5,-70,35,0,0,1);
        foreach(double y in new double[]{-70,-10})
        {
            Rectangle(-52.5,y-5,35,-37.5,y+5,35);
            Rectangle(37.5,y-5,35,52.5,y+5,35);
        }
        EndSketch("V6 four full-depth PCB pads centers x42.5 y-70 and -10");
        IFeature pads=model.FeatureManager.FeatureExtrusion3(true,false,true,0,0,M(10),0,false,false,false,false,0,0,false,false,false,false,true,true,true,0,0,false);
        Check(pads!=null,"PCB support pads failed"); pads.Name="V6 solid screw pads with 5 mm inward ears";
        model.ClearSelection2(true);
        double beforeDrilling=VolumeMm3();
        FaceSketch(-42.5,-70,35,0,0,1);
        foreach(double x in new double[]{-42.5,42.5}) foreach(double y in new double[]{-70,-10}) Circle(x,y,35);
        // The rear support pads overlap half of the existing two chassis
        // holes. Re-cut these original holes to preserve their full bores.
        Circle(-47.5,-15,35); Circle(47.5,-15,35);
        EndSketch("V6 PCB holes 85 x 60 mm and retained chassis holes");
        Cut(10,"V6 four PCB M3 holes through reinforced beam pads");
        Check(model.ForceRebuild3(false),"V6 rebuild failed");
        double actualRemoval=beforeDrilling-VolumeMm3();
        Check(Math.Abs(actualRemoval-50*Math.PI*1.7*1.7)<0.1,"PCB drilling check mismatch: "+actualRemoval);
        object[] bodies=(object[])((IPartDoc)model).GetBodies2(0,true);
        Check(bodies!=null && bodies.Length==1,"V6 must remain one connected solid");
        double[] bounds=(double[])((IBody2)bodies[0]).GetBodyBox();
        Check(Math.Abs(bounds[1]+.08)<1e-7,"Beam tip must be at y=-80 mm");
        IModelView view=(IModelView)model.ActiveView;
        double av=1/Math.Sqrt(2),bv=1/Math.Sqrt(6),cv=1/Math.Sqrt(3);
        view.Orientation3=(MathTransform)math.CreateTransform(new double[]{av,-bv,cv,av,bv,-cv,0,2*bv,cv,0,0,0,1,0,0,0});
        model.ViewZoomtofit2(); model.GraphicsRedraw2();
        string saved=Save(native);
        Check(model.SaveBMP(Path.Combine(folder,basename+"_preview.bmp"),1400,900),"V6 preview failed");
        saved+="\n"+Save(output);
        return saved+"\nPCB hole centers mm: (-42.5,-70), (42.5,-70), (-42.5,-10), (42.5,-10), z=35\nBeam length80, center spacing95, PCB pitch85x60, hole diameter3.4 mm\nSolid count1, volume mm3="+VolumeMm3().ToString("F3");
    }
    public static string SolidBeamsAndLowerSlots(string folder,string desktopFolder)
    {
        sw=Connect(); model=sw.ActiveDoc as IModelDoc2;
        Check(model!=null && model.GetTitle().StartsWith("F_car_battery_box_V6_board_on_beams",StringComparison.OrdinalIgnoreCase),"Expected active V6 part");
        string basename="F_car_battery_box_V7_solid_beams_10mm";
        string native=Path.Combine(folder,basename+".SLDPRT");
        string output=Path.Combine(desktopFolder,basename+".3mf");
        Check(!File.Exists(native) && !File.Exists(output),"V7 output already exists");
        Check(((IPartDoc)model).FeatureByName("V7 solid longitudinal beams")==null,"V7 already in progress");
        sketches=model.SketchManager; math=(IMathUtility)sw.GetMathUtility();
        double initial=VolumeMm3();
        // Add material only inside the current external rail envelopes.
        // Reopen the original bores below, leaving roof edits untouched.
        FaceSketch(-49,-40,35,0,0,1);
        Rectangle(-52.5,-80,35,-42.5,0,35);
        Rectangle(42.5,-80,35,52.5,0,35);
        EndSketch("V7 retain rail envelopes 10 x 10 length 80 mm");
        IFeature f=model.FeatureManager.FeatureExtrusion3(true,false,true,0,0,M(10),0,false,false,false,false,0,0,false,false,false,false,true,true,true,0,0,false);
        Check(f!=null,"Solid beam fill failed"); f.Name="V7 solid longitudinal beams";
        model.ClearSelection2(true);
        FaceSketch(0,-32.5,35,0,0,1);
        Rectangle(-42.5,-37.5,35,42.5,-27.5,35);
        EndSketch("V7 retain H crossmember envelope");
        f=model.FeatureManager.FeatureExtrusion3(true,false,true,0,0,M(10),0,false,false,false,false,0,0,false,false,false,false,true,true,true,0,0,false);
        Check(f!=null,"Solid H fill failed"); f.Name="V7 solid H crossmember";
        model.ClearSelection2(true);
        FaceSketch(-49,-40,35,0,0,1);
        foreach(double x in new double[]{-47.5,47.5}) foreach(double y in new double[]{-55,-15}) Circle(x,y,35);
        foreach(double x in new double[]{-42.5,42.5}) foreach(double y in new double[]{-70,-20,-10}) Circle(x,y,35);
        EndSketch("V7 preserve all ten existing beam M3 bores");
        Cut(10,"V7 restore chassis and PCB clearance bores");
        double solidVolume=VolumeMm3();
        Check(Math.Abs(solidVolume-initial-6156)<0.15,"Unexpected solid infill volume: "+(solidVolume-initial));
        // Restore the existing mounting plate within its original envelope,
        // then make four identical short slots at the new center height.
        FaceSketch(0,-28.5,14,0,-1,0);
        Rectangle(-55,-28.5,10,55,-28.5,25);
        EndSketch("V7 refill old mounting slots within unchanged plate");
        f=model.FeatureManager.FeatureExtrusion3(true,false,true,0,0,M(2.5),0,false,false,false,false,0,0,false,false,false,false,true,true,true,0,0,false);
        Check(f!=null,"Mounting plate refill failed"); f.Name="V7 close previous slot positions";
        model.ClearSelection2(true);
        double filledPlate=VolumeMm3();
        double slotVolume=4*(3.4*2+Math.PI*1.7*1.7)*2.5;
        Check(Math.Abs(filledPlate-solidVolume-slotVolume)<0.1,"Unexpected original plate slot volume");
        FaceSketch(0,-28.5,15,0,-1,0);
        foreach(double x in new double[]{-47.5,-20,20,47.5})
        {
            double[] a=Local(x-1,-28.5,15), b=Local(x+1,-28.5,15);
            Check(sketches.CreateSketchSlot(0,0,M(3.4),a[0],a[1],0,b[0],b[1],0,0,0,0,1,true)!=null,"V7 slot creation failed");
        }
        EndSketch("V7 slot centers z15 - 10 mm below beam underside z25");
        Cut(2.5,"V7 four horizontal slots 3.4 x 5.4 mm at 10 mm");
        Check(model.ForceRebuild3(false),"V7 rebuild failed");
        Check(Math.Abs(VolumeMm3()-solidVolume)<0.1,"Slot relocation changed total volume");
        object[] bodies=(object[])((IPartDoc)model).GetBodies2(0,true);
        Check(bodies!=null && bodies.Length==1,"V7 must remain one connected solid");
        double[] bounds=(double[])((IBody2)bodies[0]).GetBodyBox();
        double[] expected={-.055,-.08,0,.055,.045,.035};
        for(int i=0;i<6;i++) Check(Math.Abs(bounds[i]-expected[i])<1e-7,"V7 outer envelope changed");
        IModelView view=(IModelView)model.ActiveView;
        view.Orientation3=(MathTransform)math.CreateTransform(new double[]{1,0,0,0,0,-1,0,1,0,0,0,0,1,0,0,0});
        model.ViewZoomtofit2(); model.GraphicsRedraw2();
        Check(model.SaveBMP(Path.Combine(folder,basename+"_slots.bmp"),1400,900),"Slot preview failed");
        double av=1/Math.Sqrt(2),bv=1/Math.Sqrt(6),cv=1/Math.Sqrt(3);
        view.Orientation3=(MathTransform)math.CreateTransform(new double[]{av,-bv,cv,av,bv,-cv,0,2*bv,cv,0,0,0,1,0,0,0});
        model.ViewZoomtofit2(); model.GraphicsRedraw2();
        Check(model.SaveBMP(Path.Combine(folder,basename+"_preview.bmp"),1400,900),"V7 preview failed");
        string saved=Save(native)+"\n"+Save(output);
        return saved+"\nOne connected solid; volume mm3="+VolumeMm3().ToString("F3")+"; added material6156 mm3\nSlots x=-47.5,-20,20,47.5; z15, beam underside z25; width3.4 length5.4\nMainboard and chassis bores preserved; all H members solid";
    }
    public static string ShortenBeams56mm(string folder,string desktopFolder)
    {
        sw=Connect(); model=sw.ActiveDoc as IModelDoc2;
        Check(model!=null && model.GetTitle().StartsWith("F_car_battery_box_V7_solid_beams_10mm",StringComparison.OrdinalIgnoreCase),"Expected active V7 part");
        string basename="F_car_battery_box_V8_beams56_plate_fixed";
        string native=Path.Combine(folder,basename+".SLDPRT");
        string output=Path.Combine(desktopFolder,basename+".3mf");
        Check(!File.Exists(native) && !File.Exists(output),"V8 output already exists");
        Check(((IPartDoc)model).FeatureByName("V8 close chassis holes that would break beam ends")==null,"V8 already in progress");
        sketches=model.SketchManager; math=(IMathUtility)sw.GetMathUtility();
        Check(sketches.ActiveSketch==null,"Finish active user sketch before making model changes");
        double initial=VolumeMm3();
        // Preserve a closed bore after trimming: the old y=-55 row would
        // intersect the new end y=-56. Fill it and relocate to y=-50.
        FaceSketch(-50,-55,35,0,0,1);
        foreach(double x in new double[]{-47.5,47.5}) Rectangle(x-2,-57,35,x+2,-53,35);
        EndSketch("V8 refill old chassis bore row y-55 before shortening");
        IFeature f=model.FeatureManager.FeatureExtrusion3(true,false,true,0,0,M(10),0,false,false,false,false,0,0,false,false,false,false,true,true,true,0,0,false);
        Check(f!=null,"V8 old bore fill failed"); f.Name="V8 close chassis holes that would break beam ends";
        model.ClearSelection2(true);
        double twoBores=2*Math.PI*1.7*1.7*10;
        Check(Math.Abs(VolumeMm3()-initial-twoBores)<0.1,"Old chassis bore fill mismatch");
        FaceSketch(-47.5,-80,30,0,-1,0);
        Rectangle(-55,-80,25,55,-80,35);
        EndSketch("V8 shorten both free ends by 24 mm");
        Cut(24,"V8 solid beams length 56 mm preserve plate position");
        double removed=2*10*24*10+2*5*10*10-twoBores;
        Check(Math.Abs(VolumeMm3()-(initial+twoBores-removed))<0.1,"Beam trim volume mismatch");
        FaceSketch(-50,-50,35,0,0,1);
        Circle(-47.5,-50,35); Circle(47.5,-50,35);
        EndSketch("V8 chassis pair y-50 with 6 mm end distance");
        Cut(10,"V8 chassis M3 holes moved 5 mm toward battery box");
        Check(model.ForceRebuild3(false),"V8 rebuild failed");
        Check(Math.Abs(VolumeMm3()-(initial-removed))<0.1,"V8 final volume mismatch");
        object[] bodies=(object[])((IPartDoc)model).GetBodies2(0,true);
        Check(bodies!=null && bodies.Length==1,"V8 must remain one connected solid");
        IBody2 body=(IBody2)bodies[0];
        double[] bounds=(double[])body.GetBodyBox();
        double[] expected={-.055,-.056,0,.055,.045,.035};
        for(int i=0;i<6;i++) Check(Math.Abs(bounds[i]-expected[i])<1e-7,"V8 outer bounds mismatch");
        int plateFaces=0;
        foreach(IFace2 face in (object[])body.GetFaces())
        {
            double[] b=(double[])face.GetBox();
            if(Math.Abs(b[0]+.055)<1e-7 && Math.Abs(b[3]-.055)<1e-7 && Math.Abs(b[2]-.010)<1e-7 && Math.Abs(b[5]-.025)<1e-7 && Math.Abs(b[1]-b[4])<1e-7)
            {
                Check(Math.Abs(b[1]+.026)<1e-7 || Math.Abs(b[1]+.0285)<1e-7,"Plate position changed");
                double distance=(b[1]-bounds[1])*1000;
                Check(distance>=27.5-1e-5 && distance<=30+1e-5,"Plate to beam end exceeds 30 mm");
                plateFaces++;
            }
        }
        Check(plateFaces==2,"Expected two unchanged mounting plate faces");
        // Existing V3 holes form the approved PCB arrangement: beam row
        // y=-20 and roof row y=40, x=+-42.5. Verify cylindrical faces.
        int pcbBores=0;
        foreach(IFace2 face in (object[])body.GetFaces())
        {
            ISurface surf=(ISurface)face.GetSurface();
            if(!surf.IsCylinder()) continue;
            double[] cp=(double[])surf.CylinderParams;
            if(Math.Abs(cp[6]-.0017)>1e-7 || Math.Abs(Math.Abs(cp[5])-1)>1e-7) continue;
            if(Math.Abs(Math.Abs(cp[0])-.0425)<1e-7 && (Math.Abs(cp[1]+.020)<1e-7 || Math.Abs(cp[1]-.040)<1e-7)) pcbBores++;
        }
        Check(pcbBores>=4,"Approved 85 x 60 PCB bores not found");
        IModelView view=(IModelView)model.ActiveView;
        double av=1/Math.Sqrt(2),bv=1/Math.Sqrt(6),cv=1/Math.Sqrt(3);
        view.Orientation3=(MathTransform)math.CreateTransform(new double[]{av,-bv,cv,av,bv,-cv,0,2*bv,cv,0,0,0,1,0,0,0});
        model.ViewZoomtofit2(); model.GraphicsRedraw2();
        Check(model.SaveBMP(Path.Combine(folder,basename+"_preview.bmp"),1400,900),"V8 preview failed");
        string saved=Save(native)+"\n"+Save(output);
        return saved+"\nBeam length56 mm; plate y-28.5 to -26 unchanged; free end y-56; distances27.5/30 mm.\nPCB x+-42.5,y-20/40, pitch85x60 mm. Chassis end pair shifted y-55 to -50.\nOne connected solid; volume mm3="+VolumeMm3().ToString("F3");
    }
    static IBody2 TemporaryBox(double x0,double y0,double z0,double x1,double y1,double z1)
    {
        IModeler modeller=(IModeler)sw.GetModeler();
        IBody2 body=(IBody2)modeller.CreateBodyFromBox3(new double[]{M((x0+x1)/2),M((y0+y1)/2),M(z0),0,0,1,M(x1-x0),M(y1-y0),M(z1-z0)});
        Check(body!=null,"Temporary clipping box failed");
        return body;
    }
    public static string WidenBoxToBeamEdges105mm(string folder,string desktopFolder)
    {
        sw=Connect(); model=sw.ActiveDoc as IModelDoc2;
        Check(model!=null && model.GetTitle().StartsWith("F_car_battery_box_V9_box_forward26",StringComparison.OrdinalIgnoreCase),"Expected active V9 source");
        Check(model.SketchManager.ActiveSketch==null,"Finish active user sketch first");
        string basename="F_car_battery_box_V10_width105_aligned_H";
        string native=Path.Combine(folder,basename+".SLDPRT"),output=Path.Combine(desktopFolder,basename+".3mf");
        Check(!File.Exists(native) && !File.Exists(output),"V10 output already exists");
        Check(((IPartDoc)model).FeatureByName("V10 left box wall aligned to beam x-52.5")==null,"V10 already in progress");
        sketches=model.SketchManager; math=(IMathUtility)sw.GetMathUtility();
        double initial=VolumeMm3();
        // Add 2.5 mm on each side, without scaling or moving any holes.
        FaceSketch(-50,5,17,-1,0,0);
        Rectangle(-50,-26,0,-50,19,35);
        EndSketch("V10 extend left box side by 2.5 mm"); Boss(2.5,"V10 left box wall aligned to beam x-52.5");
        FaceSketch(50,5,17,1,0,0);
        Rectangle(50,-26,0,50,19,35);
        EndSketch("V10 extend right box side by 2.5 mm"); Boss(2.5,"V10 right box wall aligned to beam x52.5");
        Check(Math.Abs(VolumeMm3()-initial-7875)<0.1,"Outer box widening volume mismatch");
        double beforeCavity=VolumeMm3();
        // Widen the cavity to 101 mm; preserve 2 mm walls and the rear opening.
        FaceSketch(51.5,19,17,0,1,0);
        Rectangle(-50.5,19,2,50.5,19,33);
        EndSketch("V10 rear cavity width101 height31 wall2"); Cut(43,"V10 restore all box walls to 2 mm after widening");
        Check(model.ForceRebuild3(false),"V10 rebuild failed");
        double removed=beforeCavity-VolumeMm3();
        // The original chassis holes already remove small portions of the old
        // side walls, so the cavity removes slightly less than 5*43*31 mm3.
        Check(removed>6500 && removed<=6665.1,"Unexpected widened cavity volume: "+removed);
        return FinishWidenedV10(folder,desktopFolder);
    }
    public static string FinishWidenedV10(string folder,string desktopFolder)
    {
        sw=Connect(); model=sw.ActiveDoc as IModelDoc2;
        Check(model!=null && ((IPartDoc)model).FeatureByName("V10 restore all box walls to 2 mm after widening")!=null,"Expected in-progress V10");
        math=(IMathUtility)sw.GetMathUtility();
        string basename="F_car_battery_box_V10_width105_aligned_H";
        string native=Path.Combine(folder,basename+".SLDPRT"),output=Path.Combine(desktopFolder,basename+".3mf");
        Check(!File.Exists(native) && !File.Exists(output),"V10 output already exists");
        object[] bodies=(object[])((IPartDoc)model).GetBodies2(0,true);
        Check(bodies!=null && bodies.Length==1,"V10 must remain one connected solid");
        IBody2 body=(IBody2)bodies[0];
        double[] bounds=(double[])body.GetBodyBox();
        double[] expected={-.055,-.056,0,.055,.019,.035};
        for(int i=0;i<6;i++) Check(Math.Abs(bounds[i]-expected[i])<1e-7,"V10 total bounds changed unexpectedly");
        int outerWalls=0,innerWalls=0,pcbBores=0,chassisBores=0;
        foreach(IFace2 face in (object[])body.GetFaces())
        {
            double[] b=(double[])face.GetBox();
            if(Math.Abs(b[0]-b[3])<1e-7)
            {
                // Aligned coplanar beam and box sides may merge into one face.
                if(Math.Abs(Math.Abs(b[0])-.0525)<1e-7 && b[1]<=-.026+1e-7 && Math.Abs(b[4]-.019)<1e-7 && Math.Abs(b[2])<1e-7 && Math.Abs(b[5]-.035)<1e-7) outerWalls++;
                if(Math.Abs(Math.Abs(b[0])-.0505)<1e-7 && Math.Abs(b[1]+.024)<1e-7 && Math.Abs(b[4]-.019)<1e-7 && Math.Abs(b[2]-.002)<1e-7 && Math.Abs(b[5]-.033)<1e-7) innerWalls++;
            }
            ISurface surface=(ISurface)face.GetSurface();
            if(!surface.IsCylinder()) continue;
            double[] p=(double[])surface.CylinderParams;
            if(Math.Abs(p[6]-.0017)>1e-7 || Math.Abs(Math.Abs(p[5])-1)>1e-7) continue;
            if(Math.Abs(Math.Abs(p[0])-.0425)<1e-7 && (Math.Abs(p[1]+.046)<1e-7 || Math.Abs(p[1]-.014)<1e-7)) pcbBores++;
            if(Math.Abs(Math.Abs(p[0])-.0475)<1e-7 && (Math.Abs(p[1]+.050)<1e-7 || Math.Abs(p[1]+.015)<1e-7)) chassisBores++;
        }
        Check(outerWalls==2 && innerWalls==2,"V10 wall verification failed outer="+outerWalls+" inner="+innerWalls);
        Check(pcbBores>=4 && chassisBores>=4,"V10 mounting holes were not preserved");
        Check(52.5-(47.5+5.4/2)>=2,"Insufficient outer slot edge margin");
        IModelView view=(IModelView)model.ActiveView;
        double av=1/Math.Sqrt(2),bv=1/Math.Sqrt(6),cv=1/Math.Sqrt(3);
        view.Orientation3=(MathTransform)math.CreateTransform(new double[]{av,-bv,cv,av,bv,-cv,0,2*bv,cv,0,0,0,1,0,0,0});
        model.ViewZoomtofit2(); model.GraphicsRedraw2();
        Check(model.SaveBMP(Path.Combine(folder,basename+"_preview.bmp"),1400,900),"V10 preview failed");
        string saved=Save(native)+"\n"+Save(output);
        saved+="\n"+Save(Path.Combine(folder,basename+".STEP"));
        return saved+"\nOne connected solid; volume mm3="+VolumeMm3().ToString("F3")+"\nBox outer105x45x35, cavity101x43x31, walls2.\nSides x+-52.5 align with H beam outside; mounting plate remains110 wide.\nAll PCB/chassis hole centers unchanged; outer slot edge margin2.3 mm.";
    }
    static IBody2 TemporaryOperation(IBody2 target,IBody2 tool,int operation,string label)
    {
        int error=0;
        object[] result=(object[])target.Operations2(operation,tool,out error);
        Check(error==0 && result!=null && result.Length==1,"Temporary body operation failed: "+label+" error="+error+" count="+(result==null?0:result.Length));
        return (IBody2)result[0];
    }
    public static string MoveBoxForward26mm(string folder,string desktopFolder)
    {
        sw=Connect(); model=sw.ActiveDoc as IModelDoc2;
        Check(model!=null && model.GetTitle().StartsWith("F_car_battery_box_V8_beams56_plate_fixed",StringComparison.OrdinalIgnoreCase),"Expected current V8 source");
        Check(model.SketchManager.ActiveSketch==null,"Finish active user sketch first");
        string basename="F_car_battery_box_V9_box_forward26";
        string native=Path.Combine(folder,basename+".SLDPRT"), output=Path.Combine(desktopFolder,basename+".3mf");
        Check(!File.Exists(native) && !File.Exists(output),"V9 output already exists");
        double sourceVolume=VolumeMm3();
        object[] sourceBodies=(object[])((IPartDoc)model).GetBodies2(0,true);
        Check(sourceBodies!=null && sourceBodies.Length==1,"Source must have one body");
        IBody2 source=(IBody2)sourceBodies[0];
        math=(IMathUtility)sw.GetMathUtility();
        // Copy exact V8 geometry, including the user's custom roof slots.
        // Never move the original document or its car-mounting interface.
        IBody2 box=TemporaryOperation((IBody2)source.Copy(),TemporaryBox(-50,0,0,50,45,35),(int)swBodyOperationType_e.SWBODYINTERSECT,"extract original battery shell");
        Check(box.ApplyTransform((MathTransform)math.CreateTransform(new double[]{1,0,0,0,1,0,0,0,1,0,M(-26),0,1,0,0,0})),"Box translation failed");
        double[] boxBounds=(double[])box.GetBodyBox();
        Check(Math.Abs(boxBounds[1]+.026)<1e-7 && Math.Abs(boxBounds[4]-.019)<1e-7,"Translated box extent mismatch");
        IBody2 frame=TemporaryOperation((IBody2)source.Copy(),TemporaryBox(-60,-56,-1,60,-26,36),(int)swBodyOperationType_e.SWBODYINTERSECT,"retain forward H frame and original plate");
        IBody2 combined=TemporaryOperation(frame,box,(int)swBodyOperationType_e.SWBODYADD,"join box directly to unchanged plate");
        Check(Math.Abs(VolumeMm3()-sourceVolume)<0.001,"Source document was modified");
        string template=sw.GetUserPreferenceStringValue((int)swUserPreferenceStringValue_e.swDefaultTemplatePart);
        model=(IModelDoc2)sw.NewDocument(template,0,0,0);
        Check(model!=null,"New V9 document failed");
        sketches=model.SketchManager;
        IFeature imported=(IFeature)((IPartDoc)model).CreateFeatureFromBody3(combined,false,(int)swCreateFeatureBodyOpts_e.swCreateFeatureBodyCheck | (int)swCreateFeatureBodyOpts_e.swCreateFeatureBodySimplify);
        Check(imported!=null,"Cannot create V9 source geometry");
        imported.Name="V9 V8 shell translated 26 mm onto original car mount";
        double initial=VolumeMm3();
        // Only extend each beam inward, so the existing chassis bores stay open.
        FaceSketch(-42.5,-46,30,1,0,0);
        Rectangle(-42.5,-51,25,-42.5,-41,35);
        EndSketch("V9 left PCB pad y-46 shifted with battery shell"); Boss(5,"V9 left inward PCB ear");
        FaceSketch(42.5,-46,30,-1,0,0);
        Rectangle(42.5,-51,25,42.5,-41,35);
        EndSketch("V9 right PCB pad y-46 shifted with battery shell"); Boss(5,"V9 right inward PCB ear");
        Check(Math.Abs(VolumeMm3()-initial-1000)<0.1,"New PCB ears volume mismatch");
        FaceSketch(-42.5,-46,35,0,0,1);
        Circle(-42.5,-46,35); Circle(42.5,-46,35);
        EndSketch("V9 PCB forward row y-46 roof row y14 - pitch85x60"); Cut(10,"V9 PCB M3 beam holes diameter3.4");
        // Keep the second chassis top row in exactly the original position;
        // it now lies on the shell roof instead of the removed rear beam length.
        FaceSketch(0,-15,35,0,0,1);
        Circle(-47.5,-15,35); Circle(47.5,-15,35);
        EndSketch("V9 retain chassis top holes x47.5 y-15 z35"); Cut(10,"V9 original chassis top bores through roof edge");
        // Reopen the four unchanged horizontal slots through the new shell wall.
        // The original plate is 2.5 mm; the shell directly behind it is 2 mm.
        FaceSketch(0,-28.5,12,0,-1,0);
        foreach(double x in new double[]{-47.5,-20,20,47.5})
        {
            double[] a=Local(x-1,-28.5,15), b=Local(x+1,-28.5,15);
            Check(sketches.CreateSketchSlot(0,0,M(3.4),a[0],a[1],0,b[0],b[1],0,0,0,0,1,true)!=null,"V9 slot failed");
        }
        EndSketch("V9 unchanged car slot centers z15 - 20 mm below roof"); Cut(4.5,"V9 mount slots through plate and contacting battery wall");
        Check(model.ForceRebuild3(false),"V9 rebuild failed");
        object[] bodies=(object[])((IPartDoc)model).GetBodies2(0,true);
        Check(bodies!=null && bodies.Length==1,"V9 must have one connected solid");
        IBody2 body=(IBody2)bodies[0];
        double[] bounds=(double[])body.GetBodyBox();
        double[] expected={-.055,-.056,0,.055,.019,.035};
        for(int i=0;i<6;i++) Check(Math.Abs(bounds[i]-expected[i])<1e-7,"V9 bounds mismatch at "+i);
        int pcbBores=0,chassisBores=0,mountFaces=0;
        foreach(IFace2 face in (object[])body.GetFaces())
        {
            double[] b=(double[])face.GetBox();
            if(Math.Abs(b[0]+.055)<1e-7 && Math.Abs(b[3]-.055)<1e-7 && Math.Abs(b[1]+.0285)<1e-7 && Math.Abs(b[4]+.0285)<1e-7 && Math.Abs(b[2]-.010)<1e-7 && Math.Abs(b[5]-.025)<1e-7) mountFaces++;
            ISurface surface=(ISurface)face.GetSurface();
            if(!surface.IsCylinder()) continue;
            double[] p=(double[])surface.CylinderParams;
            if(Math.Abs(p[6]-.0017)>1e-7 || Math.Abs(Math.Abs(p[5])-1)>1e-7) continue;
            if(Math.Abs(Math.Abs(p[0])-.0425)<1e-7 && (Math.Abs(p[1]+.046)<1e-7 || Math.Abs(p[1]-.014)<1e-7)) pcbBores++;
            if(Math.Abs(Math.Abs(p[0])-.0475)<1e-7 && (Math.Abs(p[1]+.050)<1e-7 || Math.Abs(p[1]+.015)<1e-7)) chassisBores++;
        }
        Check(pcbBores>=4 && chassisBores>=4 && mountFaces==1,"V9 mounting verification failed: PCB="+pcbBores+" chassis="+chassisBores+" face="+mountFaces);
        Check(VolumeMm3()<sourceVolume,"V9 should reduce body volume");
        IModelView view=(IModelView)model.ActiveView;
        double av=1/Math.Sqrt(2),bv=1/Math.Sqrt(6),cv=1/Math.Sqrt(3);
        view.Orientation3=(MathTransform)math.CreateTransform(new double[]{av,-bv,cv,av,bv,-cv,0,2*bv,cv,0,0,0,1,0,0,0});
        model.ViewZoomtofit2(); model.GraphicsRedraw2();
        Check(model.SaveBMP(Path.Combine(folder,basename+"_preview.bmp"),1400,900),"V9 preview failed");
        string saved=Save(native)+"\n"+Save(output);
        saved+="\n"+Save(Path.Combine(folder,basename+".STEP"));
        return saved+"\nOne connected solid; volume mm3="+VolumeMm3().ToString("F3")+"\nSource unchanged volume="+sourceVolume.ToString("F3")+"\nBox100x45x35 wall2 moved26 forward; frame and plate fixed.\nCar slot x=-47.5,-20,20,47.5 z15; roof35; mount thickness4.5.\nPCB x+-42.5 y-46/14 pitch85x60; chassis x+-47.5 y-50/-15 preserved.";
    }
    public static string MovePlateWithin30mm(string folder,string desktopFolder)
    {
        sw=Connect(); model=sw.ActiveDoc as IModelDoc2;
        Check(model!=null && model.GetTitle().StartsWith("F_car_battery_box_V7_solid_beams_10mm",StringComparison.OrdinalIgnoreCase),"Expected active V7 part");
        string basename="F_car_battery_box_V8_plate_within30mm";
        string native=Path.Combine(folder,basename+".SLDPRT");
        string output=Path.Combine(desktopFolder,basename+".3mf");
        Check(!File.Exists(native) && !File.Exists(output),"V8 output already exists");
        Check(((IPartDoc)model).FeatureByName("V8 mounting plate far face 30 mm from free end")==null,"V8 already in progress");
        sketches=model.SketchManager; math=(IMathUtility)sw.GetMathUtility();
        double initial=VolumeMm3();
        // End y=-80. Both plate faces must remain within 30 mm of that end.
        FaceSketch(-50,-51,25,0,0,-1);
        Rectangle(-55,-52.5,25,55,-50,25);
        EndSketch("V8 plate y-52.5 to -50 - free end y-80");
        Boss(15,"V8 mounting plate far face 30 mm from free end");
        Check(Math.Abs(VolumeMm3()-initial-4125)<0.1,"New plate envelope mismatch");
        FaceSketch(0,-52.5,15,0,-1,0);
        foreach(double x in new double[]{-47.5,-20,20,47.5})
        {
            double[] a=Local(x-1,-52.5,15), b=Local(x+1,-52.5,15);
            Check(sketches.CreateSketchSlot(0,0,M(3.4),a[0],a[1],0,b[0],b[1],0,0,0,0,1,true)!=null,"V8 slot creation failed");
        }
        EndSketch("V8 preserved four short horizontal slots z15");
        Cut(2.5,"V8 plate M3 slots 10 mm below beams");
        double plateVolume=4125-4*(3.4*2+Math.PI*1.7*1.7)*2.5;
        Check(Math.Abs(VolumeMm3()-initial-plateVolume)<0.1,"New plate slots mismatch");
        // Remove only the old hanging plate below z25, not the H crossbar.
        FaceSketch(0,-28.5,12,0,-1,0);
        Rectangle(-55,-28.5,10,55,-28.5,25);
        EndSketch("V8 old plate envelope only below beam underside");
        Cut(2.5,"V8 remove old plate after relocation by 24 mm");
        Check(model.ForceRebuild3(false),"V8 rebuild failed");
        Check(Math.Abs(VolumeMm3()-initial)<0.1,"Plate relocation must preserve volume");
        object[] bodies=(object[])((IPartDoc)model).GetBodies2(0,true);
        Check(bodies!=null && bodies.Length==1,"V8 must remain one connected solid");
        IBody2 body=(IBody2)bodies[0];
        double[] bounds=(double[])body.GetBodyBox();
        double[] expected={-.055,-.08,0,.055,.045,.035};
        for(int i=0;i<6;i++) Check(Math.Abs(bounds[i]-expected[i])<1e-7,"V8 outer envelope changed");
        int plateFaces=0;
        foreach(IFace2 face in (object[])body.GetFaces())
        {
            double[] b=(double[])face.GetBox();
            if(Math.Abs(b[0]+.055)<1e-7 && Math.Abs(b[3]-.055)<1e-7 && Math.Abs(b[2]-.010)<1e-7 && Math.Abs(b[5]-.025)<1e-7 && Math.Abs(b[1]-b[4])<1e-7)
            {
                double distance=(b[1]-bounds[1])*1000;
                Check(distance>=27.5-1e-5 && distance<=30+1e-5,"Plate face exceeds 30 mm limit");
                plateFaces++;
            }
        }
        Check(plateFaces==2,"Expected two mounting plate faces for distance verification");
        IModelView view=(IModelView)model.ActiveView;
        double av=1/Math.Sqrt(2),bv=1/Math.Sqrt(6),cv=1/Math.Sqrt(3);
        view.Orientation3=(MathTransform)math.CreateTransform(new double[]{av,-bv,cv,av,bv,-cv,0,2*bv,cv,0,0,0,1,0,0,0});
        model.ViewZoomtofit2(); model.GraphicsRedraw2();
        Check(model.SaveBMP(Path.Combine(folder,basename+"_preview.bmp"),1400,900),"V8 preview failed");
        string saved=Save(native)+"\n"+Save(output);
        return saved+"\nPlate faces verified at 27.5 and 30 mm from y=-80 end. Center plane28.75 mm.\nOne connected solid; volume mm3="+VolumeMm3().ToString("F3")+"\nPreserved H brace, board holes, slot coordinates x=-47.5,-20,20,47.5 z15.";
    }
    public static string ExportCurrent3mf(string desktopFolder)
    {
        sw=Connect(); model=sw.ActiveDoc as IModelDoc2;
        Check(model!=null,"No active SolidWorks document");
        Check(model.GetTitle().StartsWith("F_car_battery_box_V5_hollow_H_t2",StringComparison.OrdinalIgnoreCase),"Active document changed; recheck selection");
        string target=Path.Combine(desktopFolder,"F_car_battery_box_current.3mf");
        Check(!File.Exists(target),"Current-version 3MF already exists; refusing to overwrite");
        bool hadUnsavedChanges=model.GetSaveFlag();
        model.ClearSelection2(true);
        Check(model.ForceRebuild3(false),"Current model rebuild failed");
        double volume=VolumeMm3();
        Directory.CreateDirectory(desktopFolder);
        return Save(target)+"\nSource="+model.GetTitle()+"\nIncluded unsaved changes="+hadUnsavedChanges+"\nVolume mm3="+volume.ToString("F3");
    }
    public static string ExportLatest3mf(string folder,string desktopFolder)
    {
        sw=Connect();
        string name="F_car_battery_box_V4_horizontal_slots";
        string source=Path.GetFullPath(Path.Combine(folder,name+".SLDPRT"));
        string target=Path.Combine(desktopFolder,name+".3mf");
        Check(!File.Exists(target),"3MF output already exists; refusing to overwrite");
        int errors=0,warnings=0;
        model=sw.GetOpenDocumentByName(source) as IModelDoc2;
        if(model==null) model=(IModelDoc2)sw.OpenDoc6(source,1,1,"",ref errors,ref warnings);
        Check(model!=null,"Cannot open latest part; error="+errors);
        sw.ActivateDoc3(model.GetTitle(),false,0,ref errors);
        Directory.CreateDirectory(desktopFolder);
        return Save(target);
    }
    static double VolumeMm3()
    {
        return ((IMassProperty)model.Extension.CreateMassProperty()).Volume*1e9;
    }
    static void InternalPocket(string name)
    {
        IFeature f=model.FeatureManager.FeatureCut4(true,false,false,0,0,M(6),0,false,false,false,false,0,0,false,false,false,false,false,true,true,false,false,false,3,M(2),true,false);
        Check(f!=null,"Internal pocket failed: "+name);
        f.Name=name; model.ClearSelection2(true);
    }
    public static string BuildV5HollowH(string folder)
    {
        sw=Connect();
        string basename="F_car_battery_box_V5_hollow_H_t2";
        string native=Path.Combine(folder,basename+".SLDPRT");
        Check(!File.Exists(native),"V5 output already exists");
        string source=Path.GetFullPath(Path.Combine(folder,"F_car_battery_box_V4_horizontal_slots.SLDPRT"));
        int errors=0,warnings=0;
        model=sw.GetOpenDocumentByName(source) as IModelDoc2;
        if(model==null) model=(IModelDoc2)sw.OpenDoc6(source,1,1,"",ref errors,ref warnings);
        Check(model!=null,"V4 source could not be opened");
        sw.ActivateDoc3(model.GetTitle(),false,0,ref errors);
        Check(((IPartDoc)model).FeatureByName("H crossmember 85 mm") == null,"H revision already in memory; review before repeating");
        sketches=model.SketchManager; math=(IMathUtility)sw.GetMathUtility();
        double startVolume=VolumeMm3();

        FaceSketch(-42.5,-32.5,30,1,0,0);
        Rectangle(-42.5,-37.5,25,-42.5,-27.5,35);
        EndSketch("H crossmember 10 x 10 mm at beam midpoint");
        Boss(85,"H crossmember 85 mm");
        Check(Math.Abs(VolumeMm3()-startVolume-8500)<0.1,"Crossmember volume mismatch");

        // Hollow the longitudinal beams while retaining full-depth local
        // screw collars at y=-55 and y=-15, plus the PCB mounting-ear zone.
        FaceSketch(-47.5,-40,35,0,0,1);
        foreach(double x in new double[]{-47.5,47.5})
        {
            Rectangle(x-3,-65,35,x+3,-59,35);
            Rectangle(x-3,-51,35,x+3,-24,35);
            Rectangle(x-3,-11,35,x+3,-2,35);
        }
        EndSketch("Beam cavities 6 mm wide preserving screw collars");
        InternalPocket("Longitudinal beam walls 2 mm - inner height 6 mm");
        Check(Math.Abs(VolumeMm3()-(startVolume+8500-3024))<0.1,"Longitudinal cavities did not leave 2 mm walls");
        return CompleteV5Frame(folder,startVolume);
    }
    public static string RepairAndFinishV5(string folder)
    {
        sw=Connect(); model=(IModelDoc2)sw.ActiveDoc;
        Check(model!=null,"No active model");
        sketches=model.SketchManager; math=(IMathUtility)sw.GetMathUtility();
        IFeature f=(IFeature)((IPartDoc)model).FeatureByName("Longitudinal beam walls 2 mm - inner height 6 mm");
        Check(f!=null,"Expected in-progress hollowing feature unavailable");
        IExtrudeFeatureData2 definition=(IExtrudeFeatureData2)f.GetDefinition();
        Check(definition.AccessSelections(model,null),"Cannot access hollowing definition");
        definition.FromOffsetReverse=true;
        Check(f.ModifyDefinition(definition,model,null),"Cannot correct pocket offset direction");
        Check(model.ForceRebuild3(false),"Corrected pocket rebuild failed");
        double startVolume;
        try {
            Check(model.FeatureManager.EditRollback(3,"H crossmember 85 mm"),"Cannot inspect original source geometry");
            startVolume=VolumeMm3();
        } finally { model.FeatureManager.EditRollback(1,""); }
        Check(Math.Abs(VolumeMm3()-(startVolume+8500-3024))<0.1,"Corrected cavity volume mismatch: "+VolumeMm3());
        return CompleteV5Frame(folder,startVolume);
    }
    public static string DiagnoseV5()
    {
        sw=Connect(); model=(IModelDoc2)sw.ActiveDoc;
        IFeature f=(IFeature)((IPartDoc)model).FeatureByName("Longitudinal beam walls 2 mm - inner height 6 mm");
        IExtrudeFeatureData2 d=(IExtrudeFeatureData2)f.GetDefinition();
        string result="Current="+VolumeMm3()+"; reverse="+d.FromOffsetReverse+"; start="+d.FromOffsetDistance+"; depth="+d.GetDepth(true);
        try {
            Check(model.FeatureManager.EditRollback(3,"H crossmember 85 mm"),"Cannot inspect source volume");
            result+="\nSource volume="+VolumeMm3();
            Check(model.FeatureManager.EditRollback(3,"Longitudinal beam walls 2 mm - inner height 6 mm"),"Cannot inspect pre-pocket volume");
            result+="\nBefore cavities="+VolumeMm3();
        } finally { model.FeatureManager.EditRollback(1,""); }
        result+="\nActive="+model.GetPathName();
        return result;
    }
    static string CompleteV5Frame(string folder,double startVolume)
    {
        string basename="F_car_battery_box_V5_hollow_H_t2";
        string native=Path.Combine(folder,basename+".SLDPRT");
        Check(!File.Exists(native),"V5 already saved");
        FaceSketch(0,-32.5,35,0,0,1);
        Rectangle(-44.5,-35.5,35,44.5,-29.5,35);
        EndSketch("H crossmember cavity connects both beam cavities");
        InternalPocket("H crossmember walls 2 mm");
        Check(model.ForceRebuild3(false),"V5 final rebuild failed");
        double volume=VolumeMm3();
        Check(Math.Abs(volume-(startVolume+8500-3024-3204))<0.1,"H frame final volume mismatch");
        object[] bodies=(object[])((IPartDoc)model).GetBodies2(0,true);
        Check(bodies!=null && bodies.Length==1,"H frame must remain one connected solid");
        IModelView view=(IModelView)model.ActiveView;
        double av=1/Math.Sqrt(2),bv=1/Math.Sqrt(6),cv=1/Math.Sqrt(3);
        view.Orientation3=(MathTransform)math.CreateTransform(new double[]{av,-bv,cv,av,bv,-cv,0,2*bv,cv,0,0,0,1,0,0,0});
        model.ViewZoomtofit2(); model.GraphicsRedraw2();
        string saved=Save(native);
        Check(model.SaveBMP(Path.Combine(folder,basename+"_preview.bmp"),1400,900),"H frame preview failed");
        saved+="\n"+Save(Path.Combine(folder,basename+".STL"));
        saved+="\n"+Save(Path.Combine(folder,basename+".STEP"));
        return saved+"\nOne connected solid; volume mm3="+volume.ToString("F3")+"\nHollow beam cross-section: outer10x10 inner6x6 wall2 mm; full-depth screw collars retained";
    }
    public static string OrientAndSave(string folder)
    {
        sw=Connect(); model=(IModelDoc2)sw.ActiveDoc;
        Check(model!=null && String.Equals(Path.GetFullPath(model.GetPathName()),Path.GetFullPath(Path.Combine(folder,"F_car_battery_box_200x50x50_t3.SLDPRT")),StringComparison.OrdinalIgnoreCase),"Target part not active");
        math=(IMathUtility)sw.GetMathUtility();
        IModelView view=(IModelView)model.ActiveView;
        double a=1/Math.Sqrt(2),b=1/Math.Sqrt(6),c=1/Math.Sqrt(3);
        view.Orientation3=(MathTransform)math.CreateTransform(new double[]{-a,-b,c,a,-b,c,0,2*b,c,0,0,0,1,0,0,0});
        model.ViewZoomtofit2(); model.GraphicsRedraw2();
        Save(Path.Combine(folder,"F_car_battery_box_200x50x50_t3.SLDPRT"));
        Check(model.SaveBMP(Path.Combine(folder,"battery_box_preview.bmp"),1400,900),"Preview save failed");
        return "Saved rear-opening isometric view";
    }
    static ISldWorks sw;
    static IModelDoc2 model;
    static ISketchManager sketches;
    static IMathUtility math;
    static void Check(bool ok, string message) { if (!ok) throw new Exception(message); }
    static double M(double mm) { return mm / 1000.0; }
    static double[] Local(double x, double y, double z)
    {
        IMathPoint p = (IMathPoint)math.CreatePoint(new double[] { M(x), M(y), M(z) });
        ISketch s = (ISketch)sketches.ActiveSketch;
        return (double[])((IMathPoint)p.MultiplyTransform(s.ModelToSketchTransform)).ArrayData;
    }
    static void Rectangle(double x1,double y1,double z1,double x2,double y2,double z2)
    {
        double[] a=Local(x1,y1,z1), b=Local(x2,y2,z2);
        object segments=sketches.CreateCornerRectangle(a[0],a[1],0,b[0],b[1],0);
        Check(segments!=null,"Rectangle creation failed");
    }
    static void Circle(double x,double y,double z)
    {
        double[] p=Local(x,y,z);
        Check(sketches.CreateCircleByRadius(p[0],p[1],0,M(1.7))!=null,"Circle creation failed");
    }
    static void FaceSketch(double x,double y,double z,double nx,double ny,double nz)
    {
        model.ClearSelection2(true);
        bool ok=model.Extension.SelectByRay(M(x+nx*10),M(y+ny*10),M(z+nz*10),-nx,-ny,-nz,0.0001,2,false,0,0);
        Check(ok,"Could not select face at "+x+","+y+","+z);
        sketches.InsertSketch(true);
        Check(sketches.ActiveSketch!=null,"Could not start face sketch");
        sketches.AddToDB=true;
    }
    static void EndSketch(string name)
    {
        sketches.AddToDB=false;
        IFeature profile=(IFeature)sketches.ActiveSketch;
        profile.Name=name;
        sketches.InsertSketch(true);
        model.ClearSelection2(true);
        Check(profile.Select2(false,0),"Could not select sketch "+name);
    }
    static IFeature Boss(double depth,string name)
    {
        IFeature f=model.FeatureManager.FeatureExtrusion3(true,false,false,0,0,M(depth),0,false,false,false,false,0,0,false,false,false,false,true,true,true,0,0,false);
        Check(f!=null,"Extrusion failed: "+name); f.Name=name; model.ClearSelection2(true); return f;
    }
    static IFeature Cut(double depth,string name)
    {
        IFeature f=model.FeatureManager.FeatureCut4(true,false,false,0,0,M(depth),0,false,false,false,false,0,0,false,false,false,false,false,true,true,false,false,false,0,0,false,false);
        Check(f!=null,"Cut failed: "+name); f.Name=name; model.ClearSelection2(true); return f;
    }
    static string Save(string path)
    {
        int errors=0,warnings=0;
        bool ok=model.Extension.SaveAs(path,0,1,null,ref errors,ref warnings);
        Check(ok && errors==0 && File.Exists(path),"Save failed: "+path+" errors="+errors);
        return Path.GetFileName(path)+" (warnings="+warnings+")";
    }
    static void Slot(double x, bool vertical)
    {
        double halfStraight=3.3;
        double[] a=Local(x-(vertical?0:halfStraight),0,25-(vertical?halfStraight:0));
        double[] b=Local(x+(vertical?0:halfStraight),0,25+(vertical?halfStraight:0));
        Check(sketches.CreateSketchSlot(0,0,M(3.4),a[0],a[1],0,b[0],b[1],0,0,0,0,1,true)!=null,"Slot creation failed at x="+x);
    }
    public static string CaptureV2Front(string folder)
    {
        sw=Connect(); model=(IModelDoc2)sw.ActiveDoc;
        Check(model!=null && Path.GetFileNameWithoutExtension(model.GetPathName())=="F_car_battery_box_V2_photo_t3","V2 not active");
        math=(IMathUtility)sw.GetMathUtility();
        IModelView view=(IModelView)model.ActiveView;
        double a=1/Math.Sqrt(2),b=1/Math.Sqrt(6),c=1/Math.Sqrt(3);
        view.Orientation3=(MathTransform)math.CreateTransform(new double[]{a,-b,c,a,b,-c,0,2*b,c,0,0,0,1,0,0,0});
        model.ViewZoomtofit2(); model.GraphicsRedraw2();
        Check(model.SaveBMP(Path.Combine(folder,"F_car_battery_box_V2_front_preview.bmp"),1400,900),"Front preview failed");
        view.Orientation3=(MathTransform)math.CreateTransform(new double[]{1,0,0,0,0,-1,0,1,0,0,0,0,1,0,0,0});
        model.ViewZoomtofit2(); model.GraphicsRedraw2();
        Check(model.SaveBMP(Path.Combine(folder,"F_car_battery_box_V2_slot_check.bmp"),1400,900),"Slot preview failed");
        view.Orientation3=(MathTransform)math.CreateTransform(new double[]{a,-b,c,a,b,-c,0,2*b,c,0,0,0,1,0,0,0});
        model.ViewZoomtofit2(); model.GraphicsRedraw2();
        return "Front isometric and orthographic slot previews saved";
    }
    public static string BuildV2(string folder, bool verticalSlots)
    {
        sw=Connect();
        string template=sw.GetUserPreferenceStringValue((int)swUserPreferenceStringValue_e.swDefaultTemplatePart);
        string basename="F_car_battery_box_V2_photo_t3";
        string native=Path.Combine(folder,basename+".SLDPRT");
        Check(!File.Exists(native),"V2 already exists; refusing to overwrite");
        model=(IModelDoc2)sw.NewDocument(template,0,0,0);
        Check(model!=null,"New part failed");
        sketches=model.SketchManager; math=(IMathUtility)sw.GetMathUtility();
        IFeature plane=(IFeature)model.FirstFeature();
        while(plane!=null && plane.GetTypeName2()!="RefPlane") plane=(IFeature)plane.GetNextFeature();
        Check(plane!=null && plane.Select2(false,0),"Reference plane unavailable");
        sketches.InsertSketch(true); sketches.AddToDB=true;
        Rectangle(-100,0,0,100,50,0); EndSketch("Box footprint 200 x 50 mm"); Boss(50,"Box height 50 mm");
        FaceSketch(0,50,25,0,1,0);
        Rectangle(-97,50,3,97,50,47); EndSketch("Rear opening 194 x 44 mm"); Cut(47,"Rear cavity wall 3 mm");

        // Photo reference: a central mounting crossbar and two uprights,
        // leaving openings towards the chassis for wiring and clearance.
        FaceSketch(0,0,25,0,-1,0);
        Rectangle(-97,0,3,-60,0,47); Rectangle(60,0,3,97,0,47);
        Rectangle(-50,0,3,50,0,16); Rectangle(-50,0,34,50,0,47);
        EndSketch("Chassis face openings and mounting crossbar"); Cut(3,"Open front mounting frame");
        FaceSketch(55,0,42,0,-1,0);
        Rectangle(-60,0,40,-50,0,50); Rectangle(50,0,40,60,0,50);
        EndSketch("Photo rails 10 x 10 pitch 110 provisional"); Boss(65,"Two forward rails 65 mm provisional");

        // Small inward ears accommodate the user's narrower PCB without
        // moving its measured 85 x 60 mm mounting rectangle.
        FaceSketch(-50,-20,45,1,0,0);
        Rectangle(-50,-25,40,-50,-15,50);
        EndSketch("Left inward PCB mounting ear"); Boss(12.5,"Left PCB ear 12.5 mm");
        FaceSketch(50,-20,45,-1,0,0);
        Rectangle(50,-25,40,50,-15,50);
        EndSketch("Right inward PCB mounting ear"); Boss(12.5,"Right PCB ear 12.5 mm");
        FaceSketch(-50,-31,45,1,0,0);
        Rectangle(-50,-35,47,-50,-27,50);
        EndSketch("Open frame transverse brace 8 x 3 mm"); Boss(100,"Transverse frame brace");

        FaceSketch(0,0,25,0,-1,0);
        foreach(double x in new double[]{-50,-20,20,50}) Slot(x,verticalSlots);
        EndSketch("Chassis slot centers -50 -20 20 50 mm");
        Cut(3,verticalSlots?"4x vertical slots 3.4 x 10 mm":"4x horizontal slots 3.4 x 10 mm");
        FaceSketch(0,40,50,0,0,1);
        Circle(-42.5,40,50); Circle(42.5,40,50);
        EndSketch("PCB rear holes pitch 85 mm"); Cut(3,"PCB roof holes diameter 3.4 mm");
        FaceSketch(-42.5,-20,50,0,0,1);
        Circle(-42.5,-20,50); Circle(42.5,-20,50);
        EndSketch("PCB front holes row spacing 60 mm"); Cut(10,"PCB ear holes diameter 3.4 mm");
        FaceSketch(-55,-55,50,0,0,1);
        foreach(double x in new double[]{-55,55}) foreach(double y in new double[]{-55,-15}) Circle(x,y,50);
        EndSketch("Chassis top holes 110 x 40 provisional"); Cut(10,"4x rail holes through upper and lower faces");
        Check(model.ForceRebuild3(false),"Final rebuild failed");
        object[] bodies=(object[])((IPartDoc)model).GetBodies2(0,true);
        Check(bodies!=null && bodies.Length==1,"Expected one connected solid body");
        double actual=((IMassProperty)model.Extension.CreateMassProperty()).Volume*1e9;
        double slotArea=3.4*6.6+Math.PI*1.7*1.7;
        double expected=200*50*50-194*47*44-2*37*44*3-2*100*13*3+2*10*65*10+2*12.5*10*10+100*8*3-4*slotArea*3-Math.PI*1.7*1.7*(2*3+2*10+4*10);
        Check(Math.Abs(actual-expected)<0.1,"Volume mismatch actual="+actual+" expected="+expected);
        double[] bounds=(double[])((IBody2)bodies[0]).GetBodyBox();
        Check(Math.Abs(bounds[3]-bounds[0]-.2)<1e-7 && Math.Abs(bounds[4]-bounds[1]-.115)<1e-7 && Math.Abs(bounds[5]-bounds[2]-.05)<1e-7,"Unexpected model bounds");
        IModelView view=(IModelView)model.ActiveView;
        double a=1/Math.Sqrt(2),b=1/Math.Sqrt(6),c=1/Math.Sqrt(3);
        view.Orientation3=(MathTransform)math.CreateTransform(new double[]{-a,-b,c,a,-b,c,0,2*b,c,0,0,0,1,0,0,0});
        model.ViewZoomtofit2(); model.GraphicsRedraw2();
        string saved=Save(native);
        bool preview=model.SaveBMP(Path.Combine(folder,basename+"_preview.bmp"),1400,900);
        saved+="\n"+Save(Path.Combine(folder,basename+".STEP"));
        saved+="\n"+Save(Path.Combine(folder,basename+".STL"));
        return saved+"\nOne connected solid. Volume mm3="+actual.ToString("F3")+"\nSlots: width 3.4 length 10 centers -50,-20,20,50; vertical="+verticalSlots+"\nPreview saved="+preview;
    }
    public static string BuildV3Base()
    {
        sw=Connect();
        string template=sw.GetUserPreferenceStringValue((int)swUserPreferenceStringValue_e.swDefaultTemplatePart);
        model=(IModelDoc2)sw.NewDocument(template,0,0,0);
        Check(model!=null,"New part failed");
        sketches=model.SketchManager; math=(IMathUtility)sw.GetMathUtility();
        IFeature plane=(IFeature)model.FirstFeature();
        while(plane!=null && plane.GetTypeName2()!="RefPlane") plane=(IFeature)plane.GetNextFeature();
        Check(plane!=null && plane.Select2(false,0),"Reference plane unavailable");
        sketches.InsertSketch(true); sketches.AddToDB=true;
        Rectangle(-50,0,0,50,45,0); EndSketch("V3 box footprint 100 x 45 mm"); Boss(35,"V3 box height 35 mm");
        FaceSketch(0,45,17,0,1,0);
        Rectangle(-48,45,2,48,45,33);
        EndSketch("V3 rear opening 96 x 31 mm"); Cut(43,"V3 cavity all walls 2 mm");
        FaceSketch(0,0,17,0,-1,0);
        Rectangle(-52.5,0,25,-42.5,0,35); Rectangle(42.5,0,25,52.5,0,35);
        EndSketch("V3 square beams 10 x 10 pitch 95 mm"); Boss(65,"V3 forward beam length 65 mm");
        FaceSketch(-42.5,-20,30,1,0,0);
        Rectangle(-42.5,-25,25,-42.5,-15,35);
        EndSketch("V3 left PCB ear profile"); Boss(5,"V3 left inward ear 5 mm");
        FaceSketch(42.5,-20,30,-1,0,0);
        Rectangle(42.5,-25,25,42.5,-15,35);
        EndSketch("V3 right PCB ear profile"); Boss(5,"V3 right inward ear 5 mm");
        FaceSketch(0,40,35,0,0,1);
        Circle(-42.5,40,35); Circle(42.5,40,35);
        EndSketch("V3 PCB roof pair pitch 85 mm"); Cut(2,"V3 M3 roof holes diameter 3.4 mm");
        FaceSketch(-42.5,-20,35,0,0,1);
        Circle(-42.5,-20,35); Circle(42.5,-20,35);
        EndSketch("V3 PCB forward pair row spacing 60 mm"); Cut(10,"V3 M3 PCB ear holes");
        FaceSketch(-47.5,-55,35,0,0,1);
        foreach(double x in new double[]{-47.5,47.5}) foreach(double y in new double[]{-55,-15}) Circle(x,y,35);
        EndSketch("V3 chassis top holes width 95 depth 40 mm"); Cut(10,"V3 M3 beam top bottom holes");
        Check(model.ForceRebuild3(false),"Base rebuild failed");
        model.ShowNamedView2("*Isometric",7); model.ViewZoomtofit2();
        return "V3 base complete; active="+model.GetTitle()+"; pending hanging mounting plate";
    }
    public static string FinishV3(string folder,double plateWidth,double outerX)
    {
        return FinishPlateVariant(folder,plateWidth,outerX,"F_car_battery_box_V3_100x45x35_t2",true,10);
    }
    public static string BuildV4(string folder,string desktopFolder)
    {
        BuildV3Base();
        string basename="F_car_battery_box_V4_horizontal_slots";
        string result=FinishPlateVariant(folder,110,47.5,basename,false,5.4);
        Directory.CreateDirectory(desktopFolder);
        string desktopStl=Path.Combine(desktopFolder,basename+".STL");
        Check(!File.Exists(desktopStl),"Desktop STL already exists; refusing to overwrite");
        File.Copy(Path.Combine(folder,basename+".STL"),desktopStl,false);
        return result+"\nDesktop STL="+desktopStl;
    }
    static string FinishPlateVariant(string folder,double plateWidth,double outerX,string basename,bool vertical,double slotLength)
    {
        sw=Connect(); model=(IModelDoc2)sw.ActiveDoc;
        Check(model!=null && ((IPartDoc)model).FeatureByName("V3 M3 beam top bottom holes")!=null,"V3 base not active");
        string native=Path.Combine(folder,basename+".SLDPRT");
        Check(!File.Exists(native),"V3 output exists; refusing to overwrite");
        Check(plateWidth/2-outerX-(vertical?1.7:slotLength/2)>=2,"Outer slots need at least 2 mm edge clearance");
        sketches=model.SketchManager; math=(IMathUtility)sw.GetMathUtility();
        FaceSketch(-47.5,-27,25,0,0,-1);
        Rectangle(-plateWidth/2,-28.5,25,plateWidth/2,-26,25);
        EndSketch("V3 hanging plate offset 26 thickness 2.5 mm"); Boss(15,"V3 mounting plate downward 15 mm width "+plateWidth);
        FaceSketch(0,-28.5,19,0,-1,0);
        foreach(double x in new double[]{-outerX,-20,20,outerX})
        {
            double halfStraight=(slotLength-3.4)/2;
            double[] a=Local(x-(vertical?0:halfStraight),-28.5,19-(vertical?halfStraight:0));
            double[] b=Local(x+(vertical?0:halfStraight),-28.5,19+(vertical?halfStraight:0));
            Check(sketches.CreateSketchSlot(0,0,M(3.4),a[0],a[1],0,b[0],b[1],0,0,0,0,1,true)!=null,"V3 slot creation failed");
        }
        EndSketch("V3 slots 6 mm below beam underside centers 20 and "+outerX);
        Cut(2.5,"Four "+(vertical?"vertical":"horizontal")+" slots 3.4 x "+slotLength+" mm");
        Check(model.ForceRebuild3(false),"V3 rebuild failed");
        object[] bodies=(object[])((IPartDoc)model).GetBodies2(0,true);
        Check(bodies!=null && bodies.Length==1,"V3 must be one connected solid");
        double volume=((IMassProperty)model.Extension.CreateMassProperty()).Volume*1e9;
        double expected=100*45*35-96*43*31+2*10*65*10+2*5*10*10+plateWidth*15*2.5-Math.PI*1.7*1.7*(2*2+2*10+4*10)-4*(3.4*(slotLength-3.4)+Math.PI*1.7*1.7)*2.5;
        Check(Math.Abs(volume-expected)<0.1,"V3 volume mismatch actual="+volume+" expected="+expected);
        IModelView view=(IModelView)model.ActiveView;
        double av=1/Math.Sqrt(2),bv=1/Math.Sqrt(6),cv=1/Math.Sqrt(3);
        view.Orientation3=(MathTransform)math.CreateTransform(new double[]{av,-bv,cv,av,bv,-cv,0,2*bv,cv,0,0,0,1,0,0,0});
        model.ViewZoomtofit2(); model.GraphicsRedraw2();
        string saved=Save(native);
        Check(model.SaveBMP(Path.Combine(folder,basename+"_preview.bmp"),1400,900),"V3 preview failed");
        saved+="\n"+Save(Path.Combine(folder,basename+".STEP"));
        saved+="\n"+Save(Path.Combine(folder,basename+".STL"));
        return saved+"\nSolid count=1; volume mm3="+volume.ToString("F3")+"\nPlate width="+plateWidth+"; outer slot x="+outerX;
    }
    public static string Build(string folder)
    {
        sw=Connect();
        string template=sw.GetUserPreferenceStringValue((int)swUserPreferenceStringValue_e.swDefaultTemplatePart);
        Check(File.Exists(template),"Default part template unavailable");
        string native=Path.Combine(folder,"F_car_battery_box_200x50x50_t3.SLDPRT");
        Check(!File.Exists(native),"Output already exists; refusing to overwrite");
        model=(IModelDoc2)sw.NewDocument(template,0,0,0);
        Check(model!=null,"New part failed");
        sketches=model.SketchManager;
        math=(IMathUtility)sw.GetMathUtility();
        IFeature plane=(IFeature)model.FirstFeature();
        while(plane!=null && plane.GetTypeName2()!="RefPlane") plane=(IFeature)plane.GetNextFeature();
        Check(plane!=null && plane.Select2(false,0),"Front reference plane unavailable");
        sketches.InsertSketch(true); sketches.AddToDB=true;
        Rectangle(-100,0,0,100,50,0);
        EndSketch("Box footprint 200 x 50 mm");
        Boss(50,"Box height 50 mm");
        FaceSketch(0,50,25,0,1,0);
        Rectangle(-97,50,3,97,50,47);
        EndSketch("Rear opening 194 x 44 mm");
        Cut(47,"Rear cavity - wall 3 mm");
        FaceSketch(0,0,25,0,-1,0);
        Rectangle(-47.5,0,40,-37.5,0,50);
        Rectangle(37.5,0,40,47.5,0,50);
        EndSketch("Two square arms 10 x 10 - pitch 85 mm");
        Boss(25,"Forward square arms 25 mm");
        FaceSketch(0,0,25,0,-1,0);
        foreach(double x in new double[]{-70,70}) foreach(double z in new double[]{15,35}) Circle(x,0,z);
        EndSketch("Temporary chassis holes - 140 x 20 mm");
        Cut(3,"4x M3 clearance diameter 3.4 mm");
        FaceSketch(0,40,50,0,0,1);
        Circle(-42.5,40,50); Circle(42.5,40,50);
        EndSketch("PCB rear pair - pitch 85 mm");
        Cut(3,"2x roof holes diameter 3.4 mm");
        FaceSketch(-42.5,-20,50,0,0,1);
        Circle(-42.5,-20,50); Circle(42.5,-20,50);
        EndSketch("PCB front pair - row spacing 60 mm");
        Cut(10,"2x arm holes through top and bottom 3.4 mm");
        Check(model.ForceRebuild3(false),"Final rebuild failed");
        IPartDoc part=(IPartDoc)model;
        object[] bodies=(object[])part.GetBodies2(0,true);
        Check(bodies!=null && bodies.Length==1,"Expected one connected solid body");
        double[] box=(double[])((IBody2)bodies[0]).GetBodyBox();
        IMassProperty mass=(IMassProperty)model.Extension.CreateMassProperty();
        double actualVolume=mass.Volume*1e9;
        double expectedVolume=200*50*50-194*47*44+2*10*25*10-Math.PI*1.7*1.7*(4*3+2*3+2*10);
        Check(Math.Abs(actualVolume-expectedVolume)<0.1,"Unexpected solid volume: "+actualVolume+" vs "+expectedVolume);
        model.ShowNamedView2("*Isometric",7); model.ViewZoomtofit2();
        string saved=Save(native);
        saved+="\n"+Save(Path.Combine(folder,"F_car_battery_box_200x50x50_t3.STEP"));
        saved+="\n"+Save(Path.Combine(folder,"F_car_battery_box_200x50x50_t3.STL"));
        return saved+"\nConnected solids=1\nVolume mm3="+actualVolume.ToString("F3")+"\nBody bounding box m="+String.Join(",",box);
    }
}
