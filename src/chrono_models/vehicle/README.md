Chrono::Vehicle Model Library
=============================

Data sources for some of the vehicle models available in the Chrono::Vehicle model library are listed below. For vehicle models not listed here, synthetic data set at reasonable values are used.


#### [`duro`] Duro multipurpose transport vehicle

The Duro (DUrable RObust) model shows the principle of the Bucher Duro (now built by Mowag/General Dynamics Land Systems) suspension system, which is very special. The original version was designed for the Swiss Army.

Data were taken from:
https://de.wikipedia.org/wiki/Mowag_Duro
and:
https://militaerfahrzeuge.ch/detail_11_21_198_190.html

Simple drawings at:
https://a.d-cd.net/4cdb45s-960.jpg
http://www.duro.ch/duro039.jpg

Most data is estimated.

#### [`feda`] FED Alpha multipurpose tactical vehicle

FED Alpha data are available from the Keweenaw Reseach Center:
https://www.mtu.edu/cdt/downloadable-materials/downloadable-materials.html

The 3D graphic representation is based on the CAD files.

The tire files are from the publicly available sources.

#### [`gator`] Gator utility vehicle

The Chrono::Vehicle Gator model is a modified John Deere E-Gator Utility Vehicle, model TE 4x2:

https://www.deere.com/en/gator-utility-vehicles/traditional-gators/te-4x2-electric-utility-vehicle/

Data for this model were obtained from manufacturer brochures, adjusted for the payload modifications.

Tire data are estimated from the specs of the OEM tires (front: 22.5x10-8 and rear: 25x12-9).

#### [`gclass`] Mercedes G-class

The Mercedes G-Class is a popular offroader, available since 1979. The Chrono::Vehicle model
is based on data from the manufacturer and drawings found online.

https://de.wikipedia.org/wiki/Mercedes-Benz_G-Klasse

While the original suspension system is based on elasto-kinematic elements, this Chrono model is simplified to a pure kinematic design.

#### [`hmmwv`] HMMWV (High Mobility Multipurpose Wheeled vehicle)

The Chrono::Vehicle HMMWV model is based on:

- James Aardema, "Failure Analysis of the rear ball joint on the high-mobility multipurpose wheeled vehicle (HMMWV)," U.S. Army Tank-Automotive Command, Warren, MI 48397-5000, Technical Report No. 13337.
www.dtic.mil/dtic/tr/fulltext/u2/a201894.pdf

#### [`kraz`] Kraz semi-trailer truck

This Chrono::Vehicle longhaul model implements a Krone semi-trailer towed by a Kraz 64431 semi-tractor.

Model data was extracted from manufacturer brochures:

https://autokraz.com.ua/www/downloads/catalogue.pdf

https://www.krone-trailer.com/fileadmin/media/downloads/DE/Trockenfracht-Sattelauflieger/Datenblaetter/Dry_Liner/Dry_Liner41-STG_DE.pdf

#### [`m113`] M113 tracked armored personnel carrier

The data for the Chrono::Vehicle M113 model was obtained from the following publicly available sources:

- M. Wallin, A.K. Aboubakr, P. Jayakumar, M.D. Letherwood, D.J. Gorsich, A. Hamed, and A.A. Shabana, "A comparative study of joint formulations: Application to multibody system tracked vehicles," Nonlinear Dynamics 74(3), 2013.
- M. Wallin, A.K. Aboubakr, P. Jayakumar, M.D. Letherwood, A. Hamed, and A.A. Shabana, "Chain Dynamic Formulations for Multibody System Tracked Vehicles" (Preprint), University of Illinois at Chicago, 2013. 
www.dtic.mil/cgi-bin/GetTRDoc?AD=ADA566761 ,
www.dtic.mil/dtic/tr/fulltext/u2/a566266.pdf
- Manuel F.R. Afonso, "Ride Dynamic Analysis of Tracked Vehicles," Master Thesis, Concordia University, Montreal, Quebec, Canada, 1989.
- R.H. Keays, "Analysis of Armoured-Vehicle Track Loads and Stresses, with Considerations on Alternative Track Materials," MRL General Document, MRL-GD-0022, 1988.
http://www.dtic.mil/dtic/tr/fulltext/u2/a219397.pdf
- Peter Blume and Clemens Niesner, "M113 in der Bundeswehr - Teil 1 (M113 in the Modern German Army - Part 1)," Publication No. 5032, Verlag Jochen Vollert - Tankograd Publishing, Erlangen, Germany, 2011.
https://www.tankograd.com/cms/website.php?id=/en/M-113-in-der-Bundeswehr-Teil-1.htm

#### [`man`] MAN trucks

Most data for the MAN Kat 1 truck Chrono::Vehicle models come from the book:
P. Ocker - MAN Die Allrad-Alleskönner - Heel Verlag 1999 ISBN 3-89365-705-3

Engine data is available from Deutz: www.deutz.de

Tire data is estimated from Pacejka truck tire data set.

#### [`mrole`] Multi-role vehicle

The Chrono::Vehicle mrole is a simplified model for the 'Boxer' vehicle, produced by KMW (Krauss Maffay Wegman) and RLS (Rheinmetall Landsysteme).

The engine, drivetrain, and suspension models approximate well the original design, based on data of the manufacturers.

The steering system is simplified but functional. Details of any military mrole modules are not publicly available.

https://de.wikipedia.org/wiki/GTK_Boxer
https://unitec-medienvertrieb.de/media/image/e9/d9/a5/temp5072_boxer_04_1kAogvF9yhD7Re.jpg
https://drawingdatabase.com/wp-content/uploads/2018/06/boxer_1.gif
https://drawingdatabase.com/wp-content/uploads/2018/06/boxer_2.gif

#### [`mtv`] MTV (Medium Tactical Vehicle) trucks

The data for the Chrono::Vehicle LMTV models was obtained from the following publicly available source:

OPERATOR’S INSTRUCTIONS MANUAL M1078 SERIES, 2-1/2 TON, 4x4, LIGHT MEDIUM TACTICAL VEHICLES (LMTV)

https://tfsweb.tamu.edu/uploadedFiles/TFSMain/Preparing_for_Wildfires/Fire_Department_Programs/Career_Fire_Department_Programs/Firefighter_Property_Program/TM-9-2320-365-10%20Operators%20Manual%20M1078%20series%202,5%20ton%204x4.pdf

#### [`polaris`] Polaris ATV

Data for the Chrono::Vehicle Polaris model are based in part on:

SEA, Ltd. Vehicle Dynamics Division, "Suspension (Kinematics & Compliance) and Vehicle Inertia Measurement Facility (VIMF) Test Results on 2013 Polaris MRZR," Technical Report, July 8, 2016.

SEA, Ltd. Vehicle Dynamics Division, "Dynamic Handling and Stability Test Results on 2013 Polaris MRZR," Technical Report, July 22, 2016.

#### [`uaz`] UAZ vehicles

The UAZ (Ulyanovsky Avtomobilny Zavod) bus came to life in 1965 (Type 452). Many data is avalable by the manifacturer.

https://uazdeutschland.de/wp-content/uploads/2021/02/UAZ-Buchanka-2021.pdf
https://passport-diary.com/wp-content/uploads/2019/05/uaz-buchanka-452-camper-umbau-van-conversion-1024x840.jpg

The UAZ 469 is a 'jeep' version of the 452. The Chrono::Vehicle model represents an older design. The actually sold vehicle is called 'Hunter' and has a independent suspension in the front, while the old 469 has two solid axles like the 452.

#### [`unimog`] Unimog (Universal-Motor-Geraet) truck

The Unimog 401 is a vintage truck/tractor developed after WW 2 by Boehringer and later by Daimler Benz.

Since it has a couple of fans at least in Europe some technical material can be found on the WWW. Most important was:
https://www.unimog-community.de/wp-content/uploads/2013/02/Frosch-Daten-1955.jpg
and
https://pixels.com/featured/unimog-1949-patent-art-poster-off-road-vintage-antique-uwe-stoeter.html

Suspension and steering data have been estimated.
