{
	"patcher" : 	{
		"fileversion" : 1,
		"appversion" : 		{
			"major" : 7,
			"minor" : 3,
			"revision" : 5,
			"architecture" : "x86",
			"modernui" : 1
		}
,
		"rect" : [ 84.0, 129.0, 1088.0, 641.0 ],
		"bgcolor" : [ 0.898039, 0.898039, 0.898039, 1.0 ],
		"editing_bgcolor" : [ 0.898039, 0.898039, 0.898039, 1.0 ],
		"bglocked" : 0,
		"openinpresentation" : 0,
		"default_fontsize" : 12.0,
		"default_fontface" : 0,
		"default_fontname" : "Arial",
		"gridonopen" : 1,
		"gridsize" : [ 15.0, 15.0 ],
		"gridsnaponopen" : 1,
		"objectsnaponopen" : 1,
		"statusbarvisible" : 2,
		"toolbarvisible" : 1,
		"lefttoolbarpinned" : 0,
		"toptoolbarpinned" : 0,
		"righttoolbarpinned" : 0,
		"bottomtoolbarpinned" : 0,
		"toolbars_unpinned_last_save" : 0,
		"tallnewobj" : 0,
		"boxanimatetime" : 200,
		"enablehscroll" : 1,
		"enablevscroll" : 1,
		"devicewidth" : 0.0,
		"description" : "",
		"digest" : "",
		"tags" : "",
		"style" : "",
		"subpatcher_template" : "",
		"boxes" : [ 			{
				"box" : 				{
					"id" : "obj-15",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 528.0, 95.0, 170.0, 22.0 ],
					"style" : "",
					"text" : "prepend stopband attenuation"
				}

			}
, 			{
				"box" : 				{
					"comment" : "",
					"id" : "obj-14",
					"index" : 2,
					"maxclass" : "inlet",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 528.0, 53.0, 30.0, 30.0 ],
					"style" : ""
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-13",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 421.0, 275.0, 143.0, 22.0 ],
					"style" : "",
					"text" : "stopband_attenuation 35"
				}

			}
, 			{
				"box" : 				{
					"comment" : "",
					"id" : "obj-11",
					"index" : 1,
					"maxclass" : "outlet",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 65.0, 486.0, 30.0, 30.0 ],
					"style" : ""
				}

			}
, 			{
				"box" : 				{
					"comment" : "",
					"id" : "obj-1",
					"index" : 1,
					"maxclass" : "inlet",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 89.0, 400.0, 30.0, 30.0 ],
					"style" : ""
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-7",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"patching_rect" : [ 410.0, 205.0, 58.0, 22.0 ],
					"presentation_rect" : [ 410.0, 205.0, 58.0, 22.0 ],
					"style" : "",
					"text" : "loadbang"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-2",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 410.0, 233.0, 115.0, 22.0 ],
					"presentation_rect" : [ 410.0, 233.0, 115.0, 22.0 ],
					"style" : "",
					"text" : "frequency 200 8000"
				}

			}
, 			{
				"box" : 				{
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"id" : "obj-30",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 6,
					"outlettype" : [ "", "", "", "", "", "" ],
					"patching_rect" : [ 625.0, 179.0, 91.5, 23.0 ],
					"presentation_rect" : [ 625.0, 179.0, 91.5, 23.0 ],
					"style" : "",
					"text" : "filterdetail"
				}

			}
, 			{
				"box" : 				{
					"audioframesize" : 1024,
					"domainlabel" : "Frequency (Hz)",
					"id" : "obj-35",
					"margins" : [ 8.0, 8.0, 20.0, 50.0 ],
					"maxclass" : "plot~",
					"numinlets" : 2,
					"numoutlets" : 1,
					"numplots" : 2,
					"numpoints" : 512,
					"outlettype" : [ "" ],
					"patching_rect" : [ 625.0, 209.0, 520.0, 340.0 ],
					"presentation_rect" : [ 625.0, 209.0, 520.0, 340.0 ],
					"rangelabel" : "Magnitude (dB)",
					"subplots" : [ 						{
							"color" : [ 0.4, 0.4, 0.75, 1.0 ],
							"thickness" : 1.0,
							"point_style" : "none",
							"line_style" : "linear",
							"number_style" : "none",
							"filter" : "atodb",
							"domain_start" : 0.0,
							"domain_end" : 22050.0,
							"domain_style" : "log",
							"domain_markers" : [ 22050.0, 20000.0, 19000.0, 18000.0, 17000.0, 16000.0, 15000.0, 14000.0, 13000.0, 12000.0, 11000.0, 10000.0, 9000.0, 8000.0, 7000.0, 6000.0, 5000.0, 4000.0, 3000.0, 2000.0, 1900.0, 1800.0, 1700.0, 1600.0, 1500.0, 1400.0, 1300.0, 1200.0, 1100.0, 1000.0, 900.0, 800.0, 700.0, 600.0, 500.0, 400.0, 300.0, 200.0, 190.0, 180.0, 170.0, 160.0, 150.0, 140.0, 130.0, 120.0, 110.0, 100.0, 90.0, 80.0, 70.0, 60.0, 50.0, 40.0, 30.0, 20.0, 10.0, 0.0 ],
							"domain_labels" : [ 20.0, "20", 200.0, "200", 2000.0, "2K", 20000.0, "20K" ],
							"range_start" : -132.0,
							"range_end" : 0.0,
							"range_style" : "linear",
							"range_markers" : [ 0.0, -12.0, -24.0, -36.0, -48.0, -60.0, -72.0, -84.0, -96.0, -108.0, -120.0, -132.0 ],
							"range_labels" : [ 0.0, "0", -12.0, "-12", -24.0, "-24", -36.0, "-36", -48.0, "-48", -60.0, "-60", -72.0, "-72", -84.0, "-84", -96.0, "-96", -108.0, "-108", -120.0, "-120", -132.0, "-132" ],
							"origin_x" : 0.0,
							"origin_y" : 0.0
						}
, 						{
							"color" : [ 0.7, 0.28, 0.39, 1.0 ],
							"thickness" : 1.0,
							"point_style" : "none",
							"line_style" : "linear",
							"number_style" : "none",
							"filter" : "atodb",
							"domain_start" : 0.0,
							"domain_end" : 22050.0,
							"domain_style" : "log",
							"domain_markers" : [  ],
							"domain_labels" : [  ],
							"range_start" : -132.0,
							"range_end" : 0.0,
							"range_style" : "linear",
							"range_markers" : [  ],
							"range_labels" : [  ],
							"origin_x" : 0.0,
							"origin_y" : 0.0
						}
 ]
				}

			}
, 			{
				"box" : 				{
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"id" : "obj-25",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 65.0, 440.0, 69.0, 23.0 ],
					"presentation_rect" : [ 65.0, 415.0, 69.0, 23.0 ],
					"style" : "",
					"text" : "cascade~"
				}

			}
, 			{
				"box" : 				{
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"hidden" : 1,
					"id" : "obj-23",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 7,
							"minor" : 3,
							"revision" : 5,
							"architecture" : "x86",
							"modernui" : 1
						}
,
						"rect" : [ 109.0, 154.0, 640.0, 480.0 ],
						"bgcolor" : [ 0.898039, 0.898039, 0.898039, 1.0 ],
						"editing_bgcolor" : [ 0.898039, 0.898039, 0.898039, 1.0 ],
						"bglocked" : 0,
						"openinpresentation" : 0,
						"default_fontsize" : 12.0,
						"default_fontface" : 0,
						"default_fontname" : "Arial",
						"gridonopen" : 1,
						"gridsize" : [ 15.0, 15.0 ],
						"gridsnaponopen" : 1,
						"objectsnaponopen" : 1,
						"statusbarvisible" : 2,
						"toolbarvisible" : 1,
						"lefttoolbarpinned" : 0,
						"toptoolbarpinned" : 0,
						"righttoolbarpinned" : 0,
						"bottomtoolbarpinned" : 0,
						"toolbars_unpinned_last_save" : 0,
						"tallnewobj" : 0,
						"boxanimatetime" : 200,
						"enablehscroll" : 1,
						"enablevscroll" : 1,
						"devicewidth" : 0.0,
						"description" : "",
						"digest" : "",
						"tags" : "",
						"style" : "",
						"subpatcher_template" : "",
						"boxes" : [ 							{
								"box" : 								{
									"fontname" : "Arial",
									"fontsize" : 12.0,
									"id" : "obj-17",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 324.0, 135.0, 205.0, 22.0 ],
									"presentation_rect" : [ 324.0, 135.0, 205.0, 22.0 ],
									"style" : "",
									"text" : "hidden 0, attr stopband_attenuation"
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Arial",
									"fontsize" : 12.0,
									"id" : "obj-16",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 134.0, 135.0, 177.0, 22.0 ],
									"presentation_rect" : [ 134.0, 135.0, 177.0, 22.0 ],
									"style" : "",
									"text" : "hidden 0, attr passband_ripple"
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Arial",
									"fontsize" : 12.0,
									"id" : "obj-15",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 64.0, 135.0, 58.0, 22.0 ],
									"presentation_rect" : [ 64.0, 135.0, 58.0, 22.0 ],
									"style" : "",
									"text" : "hidden 1"
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Arial",
									"fontsize" : 12.0,
									"id" : "obj-11",
									"maxclass" : "newobj",
									"numinlets" : 4,
									"numoutlets" : 4,
									"outlettype" : [ "bang", "bang", "bang", "" ],
									"patching_rect" : [ 50.0, 100.0, 238.0, 22.0 ],
									"presentation_rect" : [ 50.0, 100.0, 238.0, 22.0 ],
									"style" : "",
									"text" : "sel butterworth chebyshev-1 chebyshev-2"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-21",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 50.0, 40.0, 25.0, 25.0 ],
									"presentation_rect" : [ 50.0, 40.0, 25.0, 25.0 ],
									"style" : ""
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-22",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 169.0, 213.0, 25.0, 25.0 ],
									"presentation_rect" : [ 169.0, 213.0, 25.0, 25.0 ],
									"style" : ""
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-15", 0 ],
									"source" : [ "obj-11", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-16", 0 ],
									"source" : [ "obj-11", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-17", 0 ],
									"source" : [ "obj-11", 2 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-22", 0 ],
									"source" : [ "obj-15", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-22", 0 ],
									"source" : [ "obj-16", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-22", 0 ],
									"source" : [ "obj-17", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-11", 0 ],
									"source" : [ "obj-21", 0 ]
								}

							}
 ],
						"bgfillcolor_type" : "gradient",
						"bgfillcolor_color1" : [ 0.454902, 0.462745, 0.482353, 1 ],
						"bgfillcolor_color2" : [ 0.290196, 0.309804, 0.301961, 1 ],
						"bgfillcolor_color" : [ 0.290196, 0.309804, 0.301961, 1 ],
						"bgfillcolor_angle" : 270.0,
						"bgfillcolor_proportion" : 0.39
					}
,
					"patching_rect" : [ 65.0, 159.0, 39.0, 23.0 ],
					"presentation_rect" : [ 65.0, 159.0, 39.0, 23.0 ],
					"saved_object_attributes" : 					{
						"description" : "",
						"digest" : "",
						"globalpatchername" : "",
						"style" : "",
						"tags" : ""
					}
,
					"style" : "",
					"text" : "p"
				}

			}
, 			{
				"box" : 				{
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"hidden" : 1,
					"id" : "obj-10",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 3,
					"outlettype" : [ "", "", "" ],
					"patching_rect" : [ 65.0, 129.0, 163.0, 23.0 ],
					"presentation_rect" : [ 65.0, 129.0, 163.0, 23.0 ],
					"style" : "",
					"text" : "getattr topology @listen 1"
				}

			}
, 			{
				"box" : 				{
					"attr" : "response",
					"fontface" : 0,
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"id" : "obj-9",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 65.0, 300.0, 230.0, 23.0 ],
					"presentation_rect" : [ 65.0, 300.0, 230.0, 23.0 ],
					"style" : ""
				}

			}
, 			{
				"box" : 				{
					"attr" : "stopband_attenuation",
					"fontface" : 0,
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"id" : "obj-8",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 65.0, 225.0, 230.0, 23.0 ],
					"presentation_rect" : [ 65.0, 225.0, 230.0, 23.0 ],
					"style" : "",
					"text_width" : 153.0
				}

			}
, 			{
				"box" : 				{
					"attr" : "order",
					"fontface" : 0,
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"id" : "obj-6",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 65.0, 275.0, 230.0, 23.0 ],
					"presentation_rect" : [ 65.0, 275.0, 230.0, 23.0 ],
					"style" : ""
				}

			}
, 			{
				"box" : 				{
					"attr" : "frequency",
					"displaymode" : 4,
					"fontface" : 0,
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"id" : "obj-5",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 65.0, 250.0, 231.0, 23.0 ],
					"presentation_rect" : [ 65.0, 250.0, 231.0, 23.0 ],
					"style" : ""
				}

			}
, 			{
				"box" : 				{
					"attr" : "topology",
					"fontface" : 0,
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"id" : "obj-4",
					"lock" : 1,
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 65.0, 200.0, 230.0, 23.0 ],
					"presentation_rect" : [ 65.0, 200.0, 230.0, 23.0 ],
					"style" : ""
				}

			}
, 			{
				"box" : 				{
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"id" : "obj-3",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "dictionary" ],
					"patching_rect" : [ 65.0, 345.0, 539.0, 23.0 ],
					"presentation_rect" : [ 65.0, 345.0, 539.0, 23.0 ],
					"style" : "",
					"text" : "filterdesign @frequency 255 8000 @order 23 @response bandstop @topology chebyshev-2"
				}

			}
 ],
		"lines" : [ 			{
				"patchline" : 				{
					"destination" : [ "obj-25", 0 ],
					"source" : [ "obj-1", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-23", 0 ],
					"hidden" : 1,
					"source" : [ "obj-10", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-3", 0 ],
					"hidden" : 1,
					"source" : [ "obj-10", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-3", 0 ],
					"source" : [ "obj-13", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-15", 0 ],
					"source" : [ "obj-14", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-3", 0 ],
					"source" : [ "obj-15", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-3", 0 ],
					"source" : [ "obj-2", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-8", 0 ],
					"hidden" : 1,
					"source" : [ "obj-23", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-11", 0 ],
					"source" : [ "obj-25", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-25", 0 ],
					"order" : 1,
					"source" : [ "obj-3", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-30", 0 ],
					"midpoints" : [ 74.5, 372.0, 618.0, 372.0, 618.0, 171.0, 634.5, 171.0 ],
					"order" : 0,
					"source" : [ "obj-3", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-35", 0 ],
					"source" : [ "obj-30", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-3", 0 ],
					"source" : [ "obj-4", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-3", 0 ],
					"source" : [ "obj-5", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-3", 0 ],
					"source" : [ "obj-6", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-13", 0 ],
					"order" : 0,
					"source" : [ "obj-7", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 0 ],
					"order" : 1,
					"source" : [ "obj-7", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-3", 0 ],
					"source" : [ "obj-8", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-3", 0 ],
					"source" : [ "obj-9", 0 ]
				}

			}
 ]
	}

}
